#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "inttypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "button_gpio.h"
#include "nvs.h"
#include "esp_system.h"
#include "nvs_flash.h"

#define CLOCK_595 27
#define LATCH_595 14
#define DATA_595 13
#define OE_595 4
#define LED 15
#define LOAD_165 16
#define CLK_165 17
#define GPIO_OUTPUT_PIN_SEL ((1ULL << CLOCK_595) | (1ULL << LATCH_595) | (1ULL << DATA_595) | (1ULL << OE_595) | (1ULL << LED) | (1ULL << LOAD_165) | (1ULL << CLK_165))

#define KEY1 18
#define KEY2 19
#define KEY3 21
#define KEY4 23
#define DATA165 5
#define GPIO_INPUT_PIN_SEL ((1ULL << KEY1) | (1ULL << KEY2) | (1ULL << KEY3) | (1ULL << KEY4))

#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_3
#define EXAMPLE_ADC1_CHAN2 ADC_CHANNEL_6
#define EXAMPLE_ADC1_CHAN3 ADC_CHANNEL_7
#define EXAMPLE_ADC1_CHAN4 ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN5 ADC_CHANNEL_5

static const char *TAG = "Ctrl";

uint8_t BitsSelection[4] = {0xFE, 0xFD, 0xFB, 0xF7};
uint8_t curDisplay[4] = {0};
uint8_t Dot = 0x80;
uint8_t BitsSele = 0;

// Loop Logix Variables
int counter = 0; // Initialize counter

// LSB (least significant bit) first or MSB (most significant bit) first
#define LSBFIRST 0
#define MSBFIRST 1

// Message Variables
#define MAX_MSGS 4                 // Maximum number of messages
#define MSG_SIZE 4                 // Size of each message
int activeMsg[MAX_MSGS][MSG_SIZE]; // Array of arrays to store messages
int msgCount = 0;                  // Current number of messages
int currentIndex = 0;              // Current index for display, static to preserve its value between calls

// Menu Variables
// int isMenu = 1;
uint8_t menuLevel = 0;
int temp = 0;
int alternatePump = 0;

// NVS Variables
nvs_handle_t settings;
int32_t settingVar = 0; // value will default to 0, if not set yet in NVS

uint8_t SEG8Code[] =
    {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00}; // Common anode Digital Tube Character Gallery

// Logix Variables
uint16_t Time_Cnt = 0;
uint8_t Out_Value = 0;
uint8_t Out_Cnt = 0;
uint8_t Value = 0;
uint8_t Relays = 0;

struct pumps
{
    // set auto switch
    unsigned int p1a : 1;
    unsigned int p2a : 1;
    unsigned int p3a : 1;
    // set hand switch
    unsigned int p1h : 1;
    unsigned int p2h : 1;
    unsigned int p3h : 1;
};

struct ps
{
    unsigned int ps1 : 1; // 0 bit
    unsigned int ps2 : 1; // 1 bit
};

struct pumps switchStatus;
struct pumps pumpEnable;
struct ps psStatus;
struct ps psEnable;

void updateNvsVar(void)
{
    settingVar = (pumpEnable.p1a << 0) | (pumpEnable.p2a << 1) | (pumpEnable.p3a << 2) | (pumpEnable.p1h << 3) | (pumpEnable.p2h << 4) | (pumpEnable.p3h << 5) | (psEnable.ps1 << 6) | (psEnable.ps2 << 7) | (alternatePump << 8);
}

void readNvsVar(void)
{
    pumpEnable.p1a = (settingVar >> 0) & 1;
    pumpEnable.p2a = (settingVar >> 1) & 1;
    pumpEnable.p3a = (settingVar >> 2) & 1;
    pumpEnable.p1h = (settingVar >> 3) & 1;
    pumpEnable.p2h = (settingVar >> 4) & 1;
    pumpEnable.p3h = (settingVar >> 5) & 1;
    psEnable.ps1 = (settingVar >> 6) & 1;
    psEnable.ps2 = (settingVar >> 7) & 1;
    alternatePump = (settingVar >> 8) & 1;
}

void initReadNvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open("storage", NVS_READWRITE, &settings);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // Read
        printf("Reading settings from NVS ... ");
        err = nvs_get_i32(settings, "settings", &settingVar);
        switch (err)
        {
        case ESP_OK:
            printf("Done\n");
            printf("Settings = %" PRIu32 "\n", settingVar);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default:
            printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    }
    readNvsVar();
    nvs_close(settings);
}

void setNvs(void)
{
    esp_err_t err = nvs_flash_init();
    // Write
    err = nvs_open("storage", NVS_READWRITE, &settings);
    printf("Updating settings in NVS ... ");
    err = nvs_set_i32(settings, "settings", settingVar);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(settings);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(settings);
}

uint8_t Read_Inputs(void)
{
    uint8_t i;
    uint8_t Temp = 0;
    gpio_set_level(LOAD_165, 0);
    gpio_set_level(LOAD_165, 1);
    for (i = 0; i < 8; i++)
    {
        Temp <<= 1;
        gpio_set_level(CLK_165, 0);
        Temp |= gpio_get_level(DATA165);
        gpio_set_level(CLK_165, 1);
    }
    return Temp;
}

void switchSet(void)
{
    uint8_t Inputs = 0;
    Inputs = Read_Inputs();
    // Set auto
    // P1
    if (Inputs & 0x01)
    {
        switchStatus.p1a = 0;
    }
    else
    {
        switchStatus.p1a = 1;
    }

    // P2
    if (Inputs & 0x04)
    {
        switchStatus.p2a = 0;
    }
    else
    {
        switchStatus.p2a = 1;
    }
    // P3
    if (Inputs & 0x16)
    {
        switchStatus.p3a = 0;
    }
    else
    {
        switchStatus.p3a = 1;
    }
    // Set hand
    // P1
    if (Inputs & 0x02)
    {
        switchStatus.p1h = 0;
    }
    else
    {
        switchStatus.p1h = 1;
    }
    // P2
    if (Inputs & 0x08)
    {
        switchStatus.p2h = 0;
    }
    else
    {
        switchStatus.p2h = 1;
    }
    // P3
    if (Inputs & 0x32)
    {
        switchStatus.p3h = 0;
    }
    else
    {
        switchStatus.p3h = 1;
    }

    // Set PS
    // PS1
    if (Inputs & 0x64)
    {
        psStatus.ps1 = 0;
    }
    else
    {
        psStatus.ps1 = 1;
    }
    // PS2
    if (Inputs & 0x128)
    {
        psStatus.ps2 = 0;
    }
    else
    {
        psStatus.ps2 = 1;
    }
}

void shift_out(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
    uint8_t i;
    // send data (bits) to 74HC595N shift register
    for (i = 0; i < 8; i++)
    {
        if (bitOrder == LSBFIRST)
        {
            gpio_set_level(dataPin, !!(val & (1 << i)));
        }
        else
        {
            gpio_set_level(dataPin, !!(val & (1 << (7 - i))));
        }
        gpio_set_level(clockPin, 1);
        gpio_set_level(clockPin, 0);
    }
}

void Send_74HC595(uint8_t Num, uint8_t Seg, uint8_t out)
{
    shift_out(DATA_595, CLOCK_595, MSBFIRST, out);
    shift_out(DATA_595, CLOCK_595, MSBFIRST, Seg);
    shift_out(DATA_595, CLOCK_595, MSBFIRST, Num);
    gpio_set_level(LATCH_595, 0);
    gpio_set_level(LATCH_595, 1);
}

void Get_KEY_Value(int lvl)
{
    static TickType_t lastPressTime = 0;
    const TickType_t debounceTime = pdMS_TO_TICKS(50);   // Debounce time of 50 ms
    const TickType_t pressInterval = pdMS_TO_TICKS(100); // Minimum interval between presses
    TickType_t currentTime = xTaskGetTickCount();

    // Check if enough time has passed since the last press
    if (currentTime - lastPressTime < pressInterval)
    {
        return; // Exit if the minimum interval has not passed
    }

    uint8_t Value1 = 0;
    uint8_t Value2 = 0;
    const int samplesCount = 5; // Number of samples for debouncing
    int matchCount = 0;         // Count of matching samples

    // Sample the button states multiple times
    for (int i = 0; i < samplesCount; ++i)
    {
        Value1 = 0;
        if (gpio_get_level(KEY1) == 0)
        {
            Value1 |= 0x01;
        }
        if (gpio_get_level(KEY2) == 0)
        {
            Value1 |= 0x02;
        }
        if (gpio_get_level(KEY3) == 0)
        {
            Value1 |= 0x04;
        }
        if (gpio_get_level(KEY4) == 0)
        {
            Value1 |= 0x08;
        }

        vTaskDelay(debounceTime); // Wait for debounceTime between samples

        // Sample the button states again to check for consistency
        Value2 = 0;
        if (gpio_get_level(KEY1) == 0)
        {
            Value2 |= 0x01;
        }
        if (gpio_get_level(KEY2) == 0)
        {
            Value2 |= 0x02;
        }
        if (gpio_get_level(KEY3) == 0)
        {
            Value2 |= 0x04;
        }
        if (gpio_get_level(KEY4) == 0)
        {
            Value2 |= 0x08;
        }

        // Check if the button states are consistent
        if (Value1 == Value2)
        {
            matchCount++;
        }
    }

    // Proceed only if the button states were consistent in most samples
    if (matchCount >= (samplesCount / 2 + 1))
    {
        // Perform the action for the button press
        if (Value1 & 0x01)
        {
            if (lvl == 0)
            {
                menuLevel++;
            }
            else
            {
                menuLevel = 0;
            }
        }
        if (Value1 & 0x02)
        {
            if (menuLevel > 1)
            {
                menuLevel--;
            }
            else if (lvl == 0)
            {
                menuLevel++;
            }
        }
        if (Value1 & 0x04)
        {
            menuLevel++;
        }
        if (Value1 & 0x08)
        {
            if (lvl == 0)
            {
                menuLevel++;
            }
            else
            {
                temp = 1 - temp;
            }
        }

        // Update LED state based on any button press
        gpio_set_level(LED, Value1 ? 0 : 1);

        // Update the timestamp of the last accepted button press
        lastPressTime = currentTime;
    }
}

/*********************************************************
 *
 *                     Messaging System
 *
 ********************************************************/
/*       NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16
 *       CH.:0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F
 *       NO.:17,18,19,20,21,22,23,24,25,26,27,28
 *       CH.:H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,*/
const int statusMessages[][4] = {
    {28, 27, 27, 28}, // " -- " 0
    {10, 26, 25, 22}, //"AUto"  1
    {23, 27, 1, 10},  // "P-1A" 2
    {23, 27, 2, 10},  // "P-2A" 3
    {23, 27, 3, 10},  // "P-3A" 4
    {23, 27, 1, 17},  // "P-1H" 5
    {23, 27, 2, 17},  // "P-2H" 6
    {23, 27, 3, 17},  // "P-3H" 7
    {23, 1, 28, 26},  // "P1 Y" 8
    {23, 2, 28, 26},  // "P2 Y" 9
    {23, 3, 28, 26},  // "P3 Y" 10
    {23, 1, 28, 21},  // "P1 N" 11
    {23, 2, 28, 21},  // "P2 N" 12
    {23, 3, 28, 21},  // "P3 N" 13
    {10, 19, 28, 26}, // "AL Y" 14
    {10, 19, 28, 21}, // "AL N" 15
    {25, 1, 28, 26},  // "t1 Y" 16
    {25, 1, 28, 21},  // "t1 N" 17
    {25, 2, 28, 26},  // "t2 Y" 18
    {25, 2, 28, 21},  // "t2 N" 19
    {15, 24, 24, 28}  // "Err " 20
};

bool areMessagesEqual(int msg1[MSG_SIZE], int msg2[MSG_SIZE])
{
    for (int i = 0; i < MSG_SIZE; i++)
    {
        if (msg1[i] != msg2[i])
        {
            return false; // Messages are different
        }
    }
    return true; // Messages are equal
}

// Function to add a message to activeMsg if it's not already added
void addMsg(int option)
{
    int targetData[4];
    for (int i = 0; i < 4; i++)
    {
        targetData[i] = statusMessages[option][i];
    }
    // First, check if the message is already in activeMsg
    for (int i = 0; i < msgCount; i++)
    {
        if (areMessagesEqual(activeMsg[i], targetData))
        {
            return; // Exit the function if message is found
        }
    }

    // If the message is not found, add it
    if (msgCount < MAX_MSGS)
    {
        for (int i = 0; i < MSG_SIZE; i++)
        {
            activeMsg[msgCount][i] = targetData[i];
        }
        msgCount++;
    }
}

void deleteMsg(int option)
{
    // Get the message to be removed
    int targetData[4];
    for (int i = 0; i < 4; i++)
    {
        targetData[i] = statusMessages[option][i];
    }

    for (int i = 0; i < msgCount; i++)
    {
        if (areMessagesEqual(activeMsg[i], targetData))
        {
            // If the current message matches the targetData, remove it
            for (int j = i; j < msgCount - 1; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    activeMsg[j][k] = activeMsg[j + 1][k];
                }
            }
            msgCount--;
            break; // Break out of the loop after removing the first matching message
        }
    }
}

void displayRotate(void)
{
    if (msgCount > 0)
    {
        // Copy the current message to curDisplay
        for (int i = 0; i < MSG_SIZE; i++)
        {
            curDisplay[i] = activeMsg[currentIndex][i];
        }
        currentIndex = (currentIndex + 1) % msgCount; // Rotate to the next message
    }
    else
    {
        // Optionally handle the case where there are no messages
        // For example, clear curDisplay or set it to a default message
        for (int i = 0; i < MSG_SIZE; i++)
        {
            curDisplay[i] = 0; // Example of clearing curDisplay
        }
    }
}

/*********          Message System End          *********/

void outputSend(void)
{
    int px1 = curDisplay[0];
    int px2 = curDisplay[1];
    int px3 = curDisplay[2];
    int px4 = curDisplay[3];
    uint8_t tempPx[4] = {0};
    tempPx[0] = SEG8Code[px1];
    tempPx[1] = SEG8Code[px2];
    tempPx[2] = SEG8Code[px3]; //| Dot;
    tempPx[3] = SEG8Code[px4];
    Send_74HC595(tempPx[BitsSele], BitsSelection[BitsSele], Out_Value);
    BitsSele++;
    if (BitsSele == 4)
    {
        BitsSele = 0;
    }
}

void mainMenu(void)
{
    Get_KEY_Value(0);
    // Code to clear relays and screen here

    if (menuLevel > 0)
    {
        // Code to display menu here
        // Code to handle menu input
        deleteMsg(0);
        while (menuLevel > 0)
        {
            temp = 0;
            switch (menuLevel)
            {
            case 1:
                // p1Active
                while (menuLevel == 1)
                {
                    temp = (pumpEnable.p1a == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    pumpEnable.p1a = temp;
                    if (pumpEnable.p1a == 1)
                    {
                        addMsg(8);
                    }
                    else
                    {
                        deleteMsg(8);
                    }
                    if (pumpEnable.p1a == 0)
                    {
                        addMsg(11);
                    }
                    else
                    {
                        deleteMsg(11);
                    }
                }
                deleteMsg(8);
                deleteMsg(11);
                break;

            case 2:
                // p2Active
                while (menuLevel == 2)
                {
                    temp = (pumpEnable.p2a == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    pumpEnable.p2a = temp;
                    if (pumpEnable.p2a == 1)
                    {
                        addMsg(9);
                    }
                    else
                    {
                        deleteMsg(9);
                    }
                    if (pumpEnable.p2a == 0)
                    {
                        addMsg(12);
                    }
                    else
                    {
                        deleteMsg(12);
                    }
                }
                deleteMsg(9);
                deleteMsg(12);
                break;

            case 3:
                // p3Active
                while (menuLevel == 3)
                {
                    temp = (pumpEnable.p3a == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    pumpEnable.p3a = temp;
                    if (pumpEnable.p3a == 1)
                    {
                        addMsg(10);
                    }
                    else
                    {
                        deleteMsg(10);
                    }
                    if (pumpEnable.p3a == 0)
                    {
                        addMsg(13);
                    }
                    else
                    {
                        deleteMsg(13);
                    }
                }
                deleteMsg(10);
                deleteMsg(13);
                break;

            case 4:
                // alternatePump
                while (menuLevel == 4)
                {
                    temp = (alternatePump == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    alternatePump = temp;
                    if (alternatePump == 1)
                    {
                        addMsg(14);
                    }
                    else
                    {
                        deleteMsg(14);
                    }
                    if (alternatePump == 0)
                    {
                        addMsg(15);
                    }
                    else
                    {
                        deleteMsg(15);
                    }
                }
                deleteMsg(14);
                deleteMsg(15);
                break;

            case 5:
                // ps1 enable psEnable.ps1
                while (menuLevel == 5)
                {
                    temp = (psEnable.ps1 == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    psEnable.ps1 = temp;
                    if (psEnable.ps1 == 1)
                    {
                        addMsg(16);
                    }
                    else
                    {
                        deleteMsg(16);
                    }
                    if (psEnable.ps1 == 0)
                    {
                        addMsg(17);
                    }
                    else
                    {
                        deleteMsg(17);
                    }
                }
                deleteMsg(16);
                deleteMsg(17);
                break;

            case 6:
                // ps2 enable
                while (menuLevel == 6)
                {
                    temp = (psEnable.ps2 == 1) ? 1 : temp;
                    Get_KEY_Value(1);
                    psEnable.ps2 = temp;
                    if (psEnable.ps2 == 1)
                    {
                        addMsg(18);
                    }
                    else
                    {
                        deleteMsg(18);
                    }
                    if (psEnable.ps2 == 0)
                    {
                        addMsg(19);
                    }
                    else
                    {
                        deleteMsg(19);
                    }
                }
                deleteMsg(18);
                deleteMsg(19);
                break;

            default:
                // Handle unexpected menuLevel values
                break;
            }
        }
        if (menuLevel == 0)
        {
            updateNvsVar();
            setNvs();
            addMsg(0);
        }
    }
}

static bool IRAM_ATTR io_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    switchSet();
    outputSend();
    if (counter == 250)
    {
        counter = 0;
        displayRotate();
    }
    counter++;
    return pdTRUE;
}

void ioTimerInit(void)
{
    gptimer_handle_t gptimer = NULL; // Universal Timer Handle
    gptimer_config_t timer_config = {
        // Initialization parameter setting
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select Clock Source
        .direction = GPTIMER_COUNT_UP,      // Upward Counting
        .resolution_hz = 1 * 1000 * 1000,   // 1MHz, 1 tick = 1us  Setting the Timing Time
    };
    ESP_LOGI(TAG, "Start timer, stop it at alarm event");
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // Creates a general-purpose timer that returns a task handle.

    gptimer_event_callbacks_t cbs = {
        // Interrupt callback function (alrm interrupt)
        .on_alarm = io_timer_cb,
    };

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL)); // The first call to this function needs to be preceded by a call to gptimer_enable
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                               // Enable Timer Interrupt

    ESP_LOGI(TAG, "Start timer, auto-reload at alarm event");
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000, // period = 1ms
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void app_main(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(OE_595, 1);
    Send_74HC595(0, 0, 0);
    gpio_set_level(OE_595, 0);
    gpio_set_level(LED, 1);

    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = DATA165;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ioTimerInit();
    initReadNvs();
    while (1)
    {
        mainMenu();
    }
}