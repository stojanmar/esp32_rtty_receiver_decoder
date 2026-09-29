// V01a odstranim magnitude bar pustim angle bar
// DDK9 RTTY DECODER
// ESP32
//
// Core 1:
//   ADC sampling
//   Twin-passband discriminator
//   RTTY bit recovery
//   ITA2 decoding
//
// Core 0:
//   RTTY text display
//   Angle bar
//   Magnitude bar
//
// DDK9:
//   50 baud
//   16000 Hz sample rate
//   320 samples / bit
//   MARK  = 1275 Hz
//   SPACE = 1725 Hz
//   SHIFT = 450 Hz
//
// ============================================================

#include <Arduino.h>
#include "driver/adc.h"
#include "hal/adc_ll.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>

#include "TwinPassbandDiscriminatorESP32.h"

#include <Arduino_GFX_Library.h>

// ============================================================
// RTTY HAM parameters
// ============================================================

#define ADC_PIN       34
#define ADC_CHANNEL   ADC1_CHANNEL_6

#define ADC_ATTEN     ADC_ATTEN_DB_11

#define SAMPLE_RATE   16000.0f

#define RTTY_BAUD     50.0f
#define RTTY_SHIFT    450.0f

#define RTTY_MARK     1275.0f
#define RTTY_SPACE    1725.0f
#define RTTY_CENTER   1500.0f

// Exactly 320 samples per bit
static constexpr int SAMPLES_PER_BIT = 320;
static constexpr int HALF_BIT        = 160;


// ============================================================
// DISPLAY
// CYD 1.9" ST7789
//
// Physical display is used in landscape:
//   320 x 170
// ============================================================

#define GFX_BL 21

Arduino_DataBus *bus =
    new Arduino_HWSPI(
        2,                  // DC
        15,                 // CS
        14,                 // SCK
        13,                 // MOSI
        GFX_NOT_DEFINED     // MISO
    );

Arduino_GFX *gfx =
    new Arduino_ST7789(
        bus,
        GFX_NOT_DEFINED,
        1,                  // rotation
        true,               // IPS
        170,                // width in constructor
        320,                // height in constructor
        0,                  // col offset 1
        0,                  // row offset 1
        35,
        0
    );


// ============================================================
// DISPLAY GEOMETRY
// ============================================================

#define DISPLAY_W 320
#define DISPLAY_H 170

// Arduino GFX default font, text size 2
#define CHAR_W 12
#define CHAR_H 16

#define MAX_COLS (DISPLAY_W / CHAR_W)
#define MAX_ROWS (DISPLAY_H / CHAR_H)

// First row is shortened because the status bars occupy
// the upper-right part of the display.

#define FIRST_ROW_COLS 15


// ============================================================
// STATUS BAR AREA
// ============================================================

#define STATUS_LABEL_X 200

#define BAR_X          205
#define BAR_W          110
#define BAR_H          7 //5

#define ANGLE_Y        0 //2
#define MAG_Y          11

#define WHITE          0xFFFF
#define BLACK          0x0000


// ============================================================
// RTTY TEXT BUFFER
// ============================================================

char textBuffer[MAX_ROWS][MAX_COLS + 1];

int textRow    = 0;
int textColumn = 0;


// ============================================================
// CHARACTER QUEUE
//
// Core 1 -> Core 0
// ============================================================

QueueHandle_t rttyCharQueue;


// ============================================================
// DISCRIMINATOR STATUS
//
// These are "latest value" variables.
// We do NOT queue every sample.
//
// Core 1 writes them.
// Core 0 reads them.
// ============================================================

struct RTTYDisplayStatus
{
    float angle;
    float magnitude;
};

volatile RTTYDisplayStatus rttyStatus =
{
    0.0f,
    0.0f
};

portMUX_TYPE statusMux =
    portMUX_INITIALIZER_UNLOCKED;


// ============================================================
// DISCRIMINATOR
// ============================================================

TwinPassbandDiscriminatorESP32::Config cfg;

TwinPassbandDiscriminatorESP32 *discriminator;


// ============================================================
// SAMPLING TIMER
// ============================================================

hw_timer_t *sampleTimer = NULL;

volatile bool sampleReady = false;
volatile uint16_t adcSample = 2048;


// ============================================================
// RTTY STATE MACHINE
// ============================================================

enum RTTYState
{
    RTTY_WAIT_START,
    RTTY_START_CENTER,
    RTTY_DATA,
    RTTY_STOP
};

RTTYState rttyState = RTTY_WAIT_START;

int rttyTimer     = 0;
int rttyBitNumber = 0;

uint8_t rttyByte = 0;

bool rttyPrevious = false;

bool figsMode = false;


// ============================================================
// ITA2 DECODER
// ============================================================

char ita2Decode(uint8_t code)
{
    static const char letters[32] =
    {
        0,    'E',  '\n', 'A',
        ' ',  'S',  'I',  'U',
        '\r', 'D',  'R',  'J',
        'N',  'F',  'C',  'K',
        'T',  'Z',  'L',  'W',
        'H',  'Y',  'P',  'Q',
        'O',  'B',  'G',  0,
        'M',  'X',  'V',  0
    };

    static const char figures[32] =
    {
        0,    '3',  '\n', '-',
        ' ',  '\'', '8',  '7',
        '\r', '$',  '4',  0,
        ',',  '!',  ':',  '(',
        '5',  '"',  ')',  '2',
        '#',  '6',  '0',  '1',
        '9',  '?',  '&',  0,
        '.',  '/',  ';',  0
    };

    // FIGS shift
    if (code == 0x1B)
    {
        figsMode = true;
        return 0;
    }

    // LTRS shift
    if (code == 0x1F)
    {
        figsMode = false;
        return 0;
    }

    if (figsMode)
        return figures[code & 0x1F];

    return letters[code & 0x1F];
}


// ============================================================
// SEND CHARACTER FROM CORE 1 TO CORE 0
// ============================================================

void sendRTTYChar(char c)
{
    if (c == 0)
        return;

    xQueueSend(
        rttyCharQueue,
        &c,
        0
    );
}


// ============================================================
// PROCESS ONE DISCRIMINATOR SAMPLE
// ============================================================

void processRTTYSample(bool in)
{
    switch (rttyState)
    {
        // ----------------------------------------------------
        // Wait for MARK -> SPACE transition
        // ----------------------------------------------------

        case RTTY_WAIT_START:

            if (!rttyPrevious && in)
            {
                // Possible START bit

                rttyTimer = HALF_BIT;

                rttyState =
                    RTTY_START_CENTER;
            }

            break;


        // ----------------------------------------------------
        // Check center of START bit
        // ----------------------------------------------------

        case RTTY_START_CENTER:

            if (--rttyTimer <= 0)
            {
                // START must still be SPACE

                if (in)
                {
                    rttyBitNumber = 0;
                    rttyByte = 0;

                    // Move to center of DATA0

                    rttyTimer =
                        SAMPLES_PER_BIT;

                    rttyState =
                        RTTY_DATA;
                }
                else
                {
                    // False start

                    rttyState =
                        RTTY_WAIT_START;
                }
            }

            break;


        // ----------------------------------------------------
        // Receive 5 ITA2 data bits
        // ----------------------------------------------------

        case RTTY_DATA:

            if (--rttyTimer <= 0)
            {
                // DDK9 polarity handling:
                // store inverted discriminator bit

                if (!in)
                    rttyByte |=
                        (1 << rttyBitNumber);

                rttyBitNumber++;

                if (rttyBitNumber >= 5)
                {
                    // Five bits received.
                    // Next is STOP bit.

                    rttyTimer =
                        SAMPLES_PER_BIT;

                    rttyState =
                        RTTY_STOP;
                }
                else
                {
                    rttyTimer =
                        SAMPLES_PER_BIT;
                }
            }

            break;


        // ----------------------------------------------------
        // Validate STOP bit
        // ----------------------------------------------------

        case RTTY_STOP:

            if (--rttyTimer <= 0)
            {
                // STOP must be MARK = 0

                if (!in)
                {
                    // Valid character

                    char c =
                        ita2Decode(rttyByte);

                    if (c)
                    {
                        sendRTTYChar(c);
                    }
                }
                else
                {
                    // Invalid STOP bit

                    sendRTTYChar('~');
                }

                // Look for next character

                rttyState =
                    RTTY_WAIT_START;
            }

            break;
    }

    rttyPrevious = in;
}


// ============================================================
// FAST ADC READ
// ============================================================

int IRAM_ATTR local_adc1_read(int channel)
{
    uint16_t adc_value;

    adc_ll_rtc_enable_channel(
        ADC_NUM_1,
        channel
    );

    adc_ll_rtc_start_convert(
        ADC_NUM_1,
        channel
    );

    while (
        adc_ll_rtc_convert_is_done(
            ADC_NUM_1
        ) == 0
    );

    adc_value =
        adc_ll_rtc_get_convert_value(
            ADC_NUM_1
        );

    return adc_value;
}


// ============================================================
// TIMER ISR
//
// 16 kHz sampling
// ============================================================

void IRAM_ATTR onSampleTimer()
{
    adcSample =
        local_adc1_read(
            ADC_CHANNEL
        );

    sampleReady = true;
}


// ============================================================
// CLEAR TEXT BUFFER
// ============================================================

void clearTextBuffer()
{
    for (int row = 0; row < MAX_ROWS; row++)
    {
        for (int col = 0; col < MAX_COLS; col++)
            textBuffer[row][col] = ' ';

        textBuffer[row][MAX_COLS] = '\0';
    }
}


// ============================================================
// DRAW STATUS LABELS
// ============================================================

void drawStatusLabels()
{
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);

    gfx->setCursor(
        STATUS_LABEL_X,
        0
    );

    gfx->print("A");

    /*gfx->setCursor(
        STATUS_LABEL_X,
        9
    );

    gfx->print("M");*/
}


// ============================================================
// DRAW ANGLE AND MAGNITUDE BARS
// ============================================================

void drawStatusBars()
{
    float angle;
    float magnitude;

    // --------------------------------------------------------
    // Copy shared values safely
    // --------------------------------------------------------

    portENTER_CRITICAL(&statusMux);

    angle =
        rttyStatus.angle;

    /*magnitude =
        rttyStatus.magnitude;*/

    portEXIT_CRITICAL(&statusMux);


    // ========================================================
    // ANGLE BAR
    //
    // angle() is approximately:
    //
    //     -PI/4 ... +PI/4
    //
    // Center = 0
    // ========================================================

    float angleNorm =
        angle / (PI / 4.0f);

    if (angleNorm < -1.0f)
        angleNorm = -1.0f;

    if (angleNorm > 1.0f)
        angleNorm = 1.0f;


    // Clear bar

    gfx->fillRect(
        BAR_X,
        ANGLE_Y,
        BAR_W,
        BAR_H + 7, //+7 pobriše malo več navzdol
        BLACK
    );


    // Center marker

    int centerX =
        BAR_X + BAR_W / 2;

    gfx->drawFastVLine(
        centerX,
        ANGLE_Y,
        BAR_H,
        WHITE
    );


    // Bar width from center

    int angleWidth =
        (int)(
            fabsf(angleNorm) *
            (BAR_W / 2 - 2)
        );


    if (angleWidth > 0)
    {
        if (angleNorm >= 0.0f)
        {
            gfx->fillRect(
                centerX + 1,
                ANGLE_Y,
                angleWidth,
                BAR_H,
                0x07e0
            );
        }
        else
        {
            gfx->fillRect(
                centerX - angleWidth,
                ANGLE_Y,
                angleWidth,
                BAR_H,
                0x07e0
            );
        }
    }


    // ========================================================
    // MAGNITUDE BAR
    // ========================================================

    /*static float magnitudePeak =
        0.001f;


    // --------------------------------------------------------
    // Automatic peak tracking
    //
    // Fast attack
    // Slow decay
    // --------------------------------------------------------

    if (magnitude > magnitudePeak)
    {
        magnitudePeak =
            magnitude;
    }
    else
    {
        magnitudePeak *=
            0.995f;
    }


    if (magnitudePeak < 0.001f)
        magnitudePeak = 0.001f;


    float magNorm =
        magnitude / magnitudePeak;

    if (magNorm < 0.0f)
        magNorm = 0.0f;

    if (magNorm > 1.0f)
        magNorm = 1.0f;


    // Clear bar

    gfx->fillRect(
        BAR_X,
        MAG_Y,
        BAR_W,
        BAR_H,
        BLACK
    );


    int magWidth =
        (int)(
            magNorm * BAR_W
        );


    if (magWidth > 0)
    {
        gfx->fillRect(
            BAR_X,
            MAG_Y,
            magWidth,
            BAR_H,
            WHITE
        );
    }*/
}


// ============================================================
// REDRAW COMPLETE TEXT SCREEN
// ============================================================

void redrawText()
{
    gfx->fillScreen(BLACK);

    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);

    for (int row = 0; row < MAX_ROWS; row++)
    {
        gfx->setCursor(
            0,
            row * CHAR_H
        );

        gfx->print(
            textBuffer[row]
        );
    }

    // Status area is redrawn after the text

    //drawStatusLabels();
    drawStatusBars();
}


// ============================================================
// SCROLL TEXT
// ============================================================

void scrollText()
{
    for (int row = 0;
         row < MAX_ROWS - 1;
         row++)
    {
        for (int col = 0;
             col < MAX_COLS;
             col++)
        {
            textBuffer[row][col] =
                textBuffer[row + 1][col];
        }

        textBuffer[row][MAX_COLS] =
            '\0';
    }


    // Clear last row

    for (int col = 0;
         col < MAX_COLS;
         col++)
    {
        textBuffer[MAX_ROWS - 1][col] =
            ' ';
    }

    textBuffer[MAX_ROWS - 1][MAX_COLS] =
        '\0';


    textRow =
        MAX_ROWS - 1;

    textColumn =
        0;


    redrawText();
}


// ============================================================
// NEW TEXT LINE
// ============================================================

void newTextLine()
{
    textColumn = 0;

    textRow++;

    if (textRow >= MAX_ROWS)
    {
        scrollText();
    }
}


// ============================================================
// DISPLAY ONE RTTY CHARACTER
// ============================================================

void displayRTTYChar(char c)
{
    // --------------------------------------------------------
    // Carriage return
    // --------------------------------------------------------

    if (c == '\r')
    {
        textColumn = 0;
        return;
    }


    // --------------------------------------------------------
    // New line
    // --------------------------------------------------------

    if (c == '\n')
    {
        newTextLine();
        return;
    }


    // --------------------------------------------------------
    // Ignore other control characters
    // --------------------------------------------------------

    if ((uint8_t)c < 32)
        return;


    // --------------------------------------------------------
    // Determine available columns on this row
    // --------------------------------------------------------

    int maxColsThisRow =
        MAX_COLS;

    if (textRow == 0)
        maxColsThisRow =
            FIRST_ROW_COLS;


    // --------------------------------------------------------
    // Wrap
    // --------------------------------------------------------

    if (textColumn >= maxColsThisRow)
    {
        newTextLine();

        maxColsThisRow =
            MAX_COLS;
    }


    // --------------------------------------------------------
    // Store character
    // --------------------------------------------------------

    textBuffer[textRow][textColumn] =
        c;

    textBuffer[textRow][MAX_COLS] =
        '\0';


    // --------------------------------------------------------
    // Draw only this character
    // --------------------------------------------------------

    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);

    gfx->setCursor(
        textColumn * CHAR_W,
        textRow * CHAR_H
    );

    gfx->print(c);


    textColumn++;
}


// ============================================================
// CORE 0 UI TASK
// ============================================================

void uiTask(void *parameter)
{
    char c;

    uint32_t lastBarUpdate =
        millis();


    for (;;)
    {
        // ----------------------------------------------------
        // Consume all waiting RTTY characters
        // ----------------------------------------------------

        while (
            xQueueReceive(
                rttyCharQueue,
                &c,
                0
            ) == pdTRUE
        )
        {
            displayRTTYChar(c);
        }


        // ----------------------------------------------------
        // Update indicators approximately 20 times/sec
        // ----------------------------------------------------

        if (
            millis() - lastBarUpdate
            >= 50
        )
        {
            lastBarUpdate =
                millis();

            drawStatusBars();
        }


        // ----------------------------------------------------
        // Give Core 0 a little breathing room
        // ----------------------------------------------------

        vTaskDelay(
            2 / portTICK_PERIOD_MS
        );
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // ========================================================
    // DISPLAY
    // ========================================================

    pinMode(
        GFX_BL,
        OUTPUT
    );

    digitalWrite(
        GFX_BL,
        HIGH
    );

    gfx->begin();

    gfx->fillScreen(
        BLACK
    );


    clearTextBuffer();

    //drawStatusLabels();

    drawStatusBars();


    // ========================================================
    // CHARACTER QUEUE
    // ========================================================

    rttyCharQueue =
        xQueueCreate(
            128,
            sizeof(char)
        );


    if (rttyCharQueue == NULL)
    {
        Serial.println(
            "ERROR: RTTY queue creation failed!"
        );

        while (1)
            delay(1000);
    }


    // ========================================================
    // START CORE 0 UI TASK
    // ========================================================

    xTaskCreatePinnedToCore(
        uiTask,
        "RTTY_UI",
        4096,
        NULL,
        1,
        NULL,
        0
    );


    // ========================================================
    // SERIAL INFORMATION
    // ========================================================

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        " DDK9 RTTY DECODER"
    );

    Serial.println(
        " ESP32 - Dual Core"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();

    Serial.printf(
        "ADC pin      : %d\n",
        ADC_PIN
    );

    Serial.printf(
        "Sample rate  : %.2f Hz\n",
        SAMPLE_RATE
    );

    Serial.printf(
        "RTTY baud    : %.2f\n",
        RTTY_BAUD
    );

    Serial.printf(
        "Samples/bit  : %d\n",
        SAMPLES_PER_BIT
    );

    Serial.printf(
        "MARK         : %.1f Hz\n",
        RTTY_MARK
    );

    Serial.printf(
        "SPACE        : %.1f Hz\n",
        RTTY_SPACE
    );

    Serial.printf(
        "SHIFT        : %.1f Hz\n",
        RTTY_SHIFT
    );

    Serial.printf(
        "CENTER       : %.1f Hz\n",
        RTTY_CENTER
    );


    // ========================================================
    // ADC
    // ========================================================

    analogReadResolution(12);

    adc1_config_width(
        ADC_WIDTH_BIT_12
    );

    adc1_config_channel_atten(
        ADC_CHANNEL,
        ADC_ATTEN
    );

    delay(10);


    int rawd =
        adc1_get_raw(
            ADC_CHANNEL
        );

    Serial.print(
        "Initial ADC   : "
    );

    Serial.println(rawd);


    // ========================================================
    // DISCRIMINATOR
    // ========================================================

    cfg.fs =
        SAMPLE_RATE;

    cfg.f0 =
        RTTY_CENTER;

    cfg.shift =
        RTTY_SHIFT;

    cfg.R =
        0.970f;

    cfg.bps =
        RTTY_BAUD;

    cfg.int_len =
        0.5f;

    cfg.trigger =
        0.1f;


    discriminator =
        new TwinPassbandDiscriminatorESP32(
            cfg
        );


    // ========================================================
    // SAMPLING TIMER
    //
    // ESP32 APB clock = 80 MHz
    //
    // divider = 40
    // timer frequency = 2 MHz
    //
    // 125 ticks = 62.5 us
    //
    // => 16000 Hz
    // ========================================================

    sampleTimer =
        timerBegin(
            0,
            40,
            true
        );


    timerAttachInterrupt(
        sampleTimer,
        &onSampleTimer,
        true
    );


    timerAlarmWrite(
        sampleTimer,
        125,
        true
    );


    timerAlarmEnable(
        sampleTimer
    );


    Serial.println();

    Serial.println(
        "Starting discriminator..."
    );

    Serial.println(
        "RTTY decoder running."
    );

    Serial.println(
        "Core 1 = decoder"
    );

    Serial.println(
        "Core 0 = display"
    );

    Serial.println();
}

// LOOP
// Arduino loop runs on Core 1.
// This is deliberately kept very close to the proven
// working decoder.
// ============================================================

void loop()
{
    if (sampleReady)
    {
        // Get sample atomically
        // ----------------------------------------------------

        noInterrupts();

        uint16_t raw = adcSample;
        sampleReady = false;

        interrupts();

        // Remove ADC DC offset
        // Proven value for this receiver:
        // 1900
        // ----------------------------------------------------

        float sample = (float)raw - 1900.0f;

        // Run discriminator
        // ----------------------------------------------------

        bool out = discriminator->run(sample);

        // DDK9 polarity inversion
        // This is the proven inversion.
        // ----------------------------------------------------

        out = !out;

        // Update display status
        // These values are only the latest values.
        // No queue is used here.
        // ----------------------------------------------------

        float angle = discriminator->angle();
        /*float high =
            discriminator->levelHighValue();
        float low =
            discriminator->levelLowValue();
        float magnitude =
            high + low;*/

        portENTER_CRITICAL(&statusMux);

        rttyStatus.angle = angle;
        /*rttyStatus.magnitude =
            magnitude;*/

        portEXIT_CRITICAL(&statusMux);

        // RTTY bit recovery
        // ----------------------------------------------------

        processRTTYSample(out);
    }
}
