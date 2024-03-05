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
#include "iot_button.h"

#define CLOCK_595 27
#define LATCH_595 14
#define DATA_595 13
#define OE_595 4
#define LED 15
#define GPIO_OUTPUT_PIN_SEL ((1ULL << CLOCK_595) | (1ULL << LATCH_595) | (1ULL << DATA_595) | (1ULL << OE_595) | (1ULL << LED) | (1ULL << LOAD_165) | (1ULL << CLK_165))
#define KEY1 GPIO_NUM_18
#define KEY2 GPIO_NUM_19
#define KEY3 GPIO_NUM_21
#define KEY4 GPIO_NUM_23
#define LOAD_165 16
#define CLK_165 17
#define DATA165 5
#define GPIO_INPUT_PIN_SEL ((1ULL << KEY1) | (1ULL << KEY2) | (1ULL << KEY3) | (1ULL << KEY4))

#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_0
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_3
#define EXAMPLE_ADC1_CHAN2 ADC_CHANNEL_6
#define EXAMPLE_ADC1_CHAN3 ADC_CHANNEL_7
#define EXAMPLE_ADC1_CHAN4 ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN5 ADC_CHANNEL_5

static const char *TAG = "Ctrl";

//Display output variables
uint8_t BitsSelection[4] = {0xFE, 0xFD, 0xFB, 0xF7};
uint8_t Dot = 0x80; // Dot for display
uint8_t BitsSele = 0;

// LSB (least significant bit) first or MSB (most significant bit) first
#define LSBFIRST 0
#define MSBFIRST 1

// Message Variables
#define MAX_MSGS 10                 // Adjust as needed
#define MSG_SIZE 5                  // Number of segments per message
#define SEGMENT_CODES 29            // Number of entries in SEG8Code
uint8_t activeMsg[MAX_MSGS][MSG_SIZE];
uint8_t curDisplay[MSG_SIZE];
uint8_t msgCount = 0;
uint8_t currentIndex = 0;
uint32_t activeMessagesMask = 0;  // Tracks messages currently active
uint32_t messagesInArrayMask = 0; // Tracks messages currently in the array
/*       NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16
 *       CH.:0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F
 *       NO.:17,18,19,20,21,22,23,24,25,26,27,28
 *       CH.:H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,*/
uint8_t statusMessages[][MSG_SIZE] = {
    {28, 27, 27, 28, 0}, // " -- " 0
    {10, 26, 25, 22, 0}, //"AUto"  1
    {23, 27, 1, 10, 0},  // "P-1A" 2
    {23, 27, 2, 10, 0},  // "P-2A" 3
    {23, 27, 3, 10, 0},  // "P-3A" 4
    {23, 27, 1, 17, 0},  // "P-1H" 5
    {23, 27, 2, 17, 0},  // "P-2H" 6
    {23, 27, 3, 17, 0},  // "P-3H" 7
    {23, 1, 28, 26, 0},  // "P1 Y" 8
    {23, 2, 28, 26, 0},  // "P2 Y" 9
    {23, 3, 28, 26, 0},  // "P3 Y" 10
    {23, 1, 28, 21, 0},  // "P1 N" 11
    {23, 2, 28, 21, 0},  // "P2 N" 12
    {23, 3, 28, 21, 0},  // "P3 N" 13
    {10, 19, 28, 26, 0}, // "AL Y" 14
    {10, 19, 28, 21, 0}, // "AL N" 15
    {25, 1, 28, 26, 0},  // "t1 Y" 16
    {25, 1, 28, 21, 0},  // "t1 N" 17
    {25, 2, 28, 26, 0},  // "t2 Y" 18
    {25, 2, 28, 21, 0},  // "t2 N" 19
    {15, 24, 24, 28, 0},  // "Err " 20
    {28, 27, 27, 28, 0} // " -- " 21
};

// Menu Variables
uint8_t menuLevel = 0;
uint8_t emenu = 0;

uint8_t SEG8Code[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00}; // Common anode Digital Tube Character Gallery

// Logix Variables

uint8_t Out_Value = 0;
uint8_t mainLogix;

// NVS Variables
nvs_handle_t nvsStorage;
uint32_t nvsSettings;

struct Pump {
    uint8_t pumpAuto;
    uint8_t pumpHand;
    uint8_t pumpEnable;
    // Add more properties as needed
};

struct PressureSwitch {
    uint8_t ps1;
    uint8_t ps2;
    uint8_t ps3;
    uint8_t ps4;
    // Add more properties as needed
};

struct PumpController {
    struct Pump pump[9];
    struct PressureSwitch psEnable;
    uint8_t alternatePump;

};

struct PumpController controller;

button_config_t gpio_btn1_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = 18,
        .active_level = 0,
    },
};
button_config_t gpio_btn2_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = 19,
        .active_level = 0,
    },
};
button_config_t gpio_btn3_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = 21,
        .active_level = 0,
    },
};
button_config_t gpio_btn4_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = 23,
        .active_level = 0,
    },
};
// END NEW VARIABLES

void initReadNvs(void *pvParameters)
{
    int32_t settingVar = 0; // value will default to 0, if not set yet in NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open("storage", NVS_READWRITE, &nvsStorage);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // Read
        printf("Reading settings from NVS ... ");
        err = nvs_get_i32(nvsStorage, "settings", &settingVar);
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

    controller.pump[1].pumpEnable = (settingVar & 0x01) ? 1 : 0;
    controller.pump[2].pumpEnable = (settingVar & 0x02) ? 1 : 0;
    controller.pump[3].pumpEnable = (settingVar & 0x04) ? 1 : 0;
    controller.pump[4].pumpEnable = (settingVar & 0x08) ? 1 : 0;
    controller.pump[5].pumpEnable = (settingVar & 0x16) ? 1 : 0;
    controller.pump[6].pumpEnable = (settingVar & 0x32) ? 1 : 0;
    controller.pump[7].pumpEnable = (settingVar & 0x64) ? 1 : 0;
    controller.pump[8].pumpEnable = (settingVar & 0x128) ? 1 : 0;
    controller.alternatePump = (settingVar & 0x256) ? 1 : 0;
    controller.psEnable.ps1 = (settingVar & 0x512) ? 1 : 0;
    controller.psEnable.ps2 = (settingVar & 0x1024) ? 1 : 0;
    controller.psEnable.ps3 = (settingVar & 0x2048) ? 1 : 0;
    controller.psEnable.ps4 = (settingVar & 0x4096) ? 1 : 0;

    nvs_close(nvsStorage);
    vTaskDelete(NULL);
}

void setNvs(void)
{
    int32_t settingVar = nvsSettings;
    esp_err_t err = nvs_flash_init();
    // Write
    err = nvs_open("storage", NVS_READWRITE, &nvsStorage);
    err = nvs_set_i32(nvsStorage, "settings", settingVar);

    // Commit written value.
    err = nvs_commit(nvsStorage);

    // Close
    nvs_close(nvsStorage);
}

void shift_out(uint8_t dataPin, uint8_t clockPin, uint8_t val)
{
    // send data (bits) to 74HC595N shift register
    for (uint8_t i = 0; i < 8; i++)
    {
        gpio_set_level(dataPin, !!(val & (1 << (7 - i))));
        gpio_set_level(clockPin, 1);
        gpio_set_level(clockPin, 0);
    }
}

void Send_74HC595(uint8_t Num, uint8_t Seg, uint8_t out)
{
    shift_out(DATA_595, CLOCK_595, out);
    shift_out(DATA_595, CLOCK_595, Seg);
    shift_out(DATA_595, CLOCK_595, Num);
    gpio_set_level(LATCH_595, 0);
    gpio_set_level(LATCH_595, 1);
}

uint8_t Read_Inputs(void)
{
    uint8_t Temp = 0;
    gpio_set_level(LOAD_165, 0);
    gpio_set_level(LOAD_165, 1);
    for (uint8_t i = 0; i < 8; i++)
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
    //switchStatus = (nvsSettings & ~(Read_Inputs()));
    //switchStatus = nvsSettings & ~((dIn & 0x01) | (dIn & 0x04) | (dIn & 0x16) | (dIn & 0x02) | (dIn & 0x08) | (dIn & 0x32) | (dIn & 0x64) | (dIn & 0x128));
}

/*********************************************************
 *
 *                     Messaging System
 *
 ********************************************************/
// Compares two messages for equality
bool areMessagesEqual(uint8_t msg1[], uint8_t msg2[]) {
    for (int i = 0; i < MSG_SIZE; i++) {
        if (msg1[i] != msg2[i]) {
            return false;
        }
    }
    return true;
}

void setMessage(int option, bool value)
{
    if (value)
    {
        if (!(activeMessagesMask & 1 << option))
        { // Check if message is not already active
            if (msgCount < MAX_MSGS)
            {
                for (int i = 0; i < MSG_SIZE; i++)
                {
                    activeMsg[msgCount][i] = statusMessages[option][i];
                }
                msgCount++;
                activeMessagesMask |= 1 << option; // Mark as active
            }
        }
    }
    else
    {
        if (activeMessagesMask & 1 << option)
        {
            // Find and remove the message from activeMsg
            for (int i = 0; i < msgCount; i++)
            {
                if (areMessagesEqual(activeMsg[i], statusMessages[option]))
                {
                    for (int j = i; j < msgCount - 1; j++)
                    {
                        for (int k = 0; k < MSG_SIZE; k++)
                        {
                            activeMsg[j][k] = activeMsg[j + 1][k]; // Shift messages down
                        }
                    }
                    msgCount--;
                    activeMessagesMask &= ~(1 << option); // Clear from mask
                    break;
                }
            }
        }
    }
}

/**
 * @brief Rotates the messages displayed on the screen.
 * 
 * This function updates the current message index and copies the corresponding message
 * to the curDisplay array. The messages are rotated in a circular manner.
 */

// Function to rotate the messages displayed on the screen
void rotateMessages(void *pvParameters)
{
    currentIndex = (currentIndex + 1) % msgCount;
    for (int i = 0; i < sizeof(curDisplay); i++)
    {
        curDisplay[i] = activeMsg[currentIndex][i];
    }
    vTaskDelete(NULL);
}

/*********          Message System End          *********/
void outputSend(void *pvParameters)
{
    while (1)
    {
        uint8_t tempPx[4] = {0};
        for (int i = 0; i < 4; i++)
        {
            if (curDisplay[4] == i + 1)
            {
                tempPx[i] = SEG8Code[curDisplay[i]] | Dot;
            }
            else
            {
                tempPx[i] = SEG8Code[curDisplay[i]];
            }
        }
        //Send_74HC595(tempPx[BitsSele], BitsSelection[BitsSele], Out_Value);
        Send_74HC595(tempPx[BitsSele], BitsSelection[BitsSele], ~(Read_Inputs()));
        BitsSele = (BitsSele + 1) % 4;
        vTaskDelay(pdTICKS_TO_MS(5));
    }
}

/**
void mainMenu(void)
{
    int buttonEvent;
    while (menuLevel>0)
    {
        switch (menuLevel)
        {
        case 0:
            if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
            {
                // Process the button event
                switch (buttonEvent)
                {
                case BUTTON1_PRESS:
                    // Handle Button 1 press
                    menuLevel++;
                    setMessage(21,0);
                    break;
                case BUTTON2_PRESS:
                    // Handle Button 2 press
                    menuLevel++;
                    setMessage(21,0);
                    break;
                case BUTTON3_PRESS:
                    // Handle Button 3 press
                    menuLevel++;
                    setMessage(21,0);
                    break;
                case BUTTON4_PRESS:
                    // Handle Button 4 press
                    menuLevel++;
                    setMessage(21,0);
                    break;
                    // Add cases for other button presses
                }
            }
            break;
        case 1:
            // p1Active
            temp = (pumpEnable.p1a == 1) ? 1 : temp;
            while (menuLevel == 1)
            {
                pumpEnable.p1a = temp;
                switch (pumpEnable.p1a)
                {
                case 0:
                    setMessage(11, 1);
                    setMessage(8, 0);
                    break;
                case 1:
                    setMessage(8, 1);
                    setMessage(11, 0);
                    break;
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel = 1;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(8, 0);
            setMessage(11, 0);
            break;

        case 2:
            // p2Active
            while (menuLevel == 2)
            {
                temp = (pumpEnable.p2a == 1) ? 1 : temp;
                pumpEnable.p2a = temp;
                if (pumpEnable.p2a == 1)
                {
                    setMessage(9, 1);
                }
                else
                {
                    setMessage(9, 0);
                }
                if (pumpEnable.p2a == 0)
                {
                    setMessage(12, 1);
                }
                else
                {
                    setMessage(12, 0);
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel--;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(9, 0);
            setMessage(12, 0);
            break;

        case 3:
            // p3Active
            while (menuLevel == 3)
            {
                temp = (pumpEnable.p3a == 1) ? 1 : temp;
                pumpEnable.p3a = temp;
                if (pumpEnable.p3a == 1)
                {
                    setMessage(10, 1);
                }
                else
                {
                    setMessage(10, 0);
                }
                if (pumpEnable.p3a == 0)
                {
                    setMessage(13, 1);
                }
                else
                {
                    setMessage(13, 0);
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel--;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(10, 0);
            setMessage(13, 0);
            break;

        case 4:
            // alternatePump
            while (menuLevel == 4)
            {
                temp = (alternatePump == 1) ? 1 : temp;
                alternatePump = temp;
                if (alternatePump == 1)
                {
                    setMessage(14, 1);
                }
                else
                {
                    setMessage(14, 0);
                }
                if (alternatePump == 0)
                {
                    setMessage(15, 1);
                }
                else
                {
                    setMessage(15, 0);
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel--;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(14, 0);
            setMessage(15, 0);
            break;

        case 5:
            // ps1 enable psEnable.ps1
            while (menuLevel == 5)
            {
                temp = (psEnable.ps1 == 1) ? 1 : temp;
                psEnable.ps1 = temp;
                if (psEnable.ps1 == 1)
                {
                    setMessage(16, 1);
                }
                else
                {
                    setMessage(16, 0);
                }
                if (psEnable.ps1 == 0)
                {
                    setMessage(17, 1);
                }
                else
                {
                    setMessage(17, 0);
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel--;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(16, 0);
            setMessage(17, 0);
            break;

        case 6:
            // ps2 enable
            temp = (psEnable.ps2 == 1) ? 1 : temp;
            while (menuLevel == 6)
            {
                psEnable.ps2 = temp;
                if (psEnable.ps2 == 1)
                {
                    setMessage(18, 1);
                }
                else
                {
                    setMessage(18, 0);
                }
                if (psEnable.ps2 == 0)
                {
                    setMessage(19, 1);
                }
                else
                {
                    setMessage(19, 0);
                }
                if (xQueueReceive(buttonEventQueue, &buttonEvent, portMAX_DELAY))
                {
                    // Process the button event
                    switch (buttonEvent)
                    {
                    case BUTTON1_PRESS:
                        // Handle Button 1 press
                        menuLevel = 99;
                        break;
                    case BUTTON2_PRESS:
                        // Handle Button 2 press
                        menuLevel--;
                        break;
                    case BUTTON3_PRESS:
                        // Handle Button 3 press
                        menuLevel++;
                        break;
                    case BUTTON4_PRESS:
                        // Handle Button 4 press
                        // printf(temp);
                        temp = 1 - temp;
                        break;
                        // Add cases for other button presses
                    }
                }
            }
            setMessage(18, 0);
            setMessage(19, 0);
            break;

        case 99:
            updateNvsVar();
            setNvs();
            setMessage(21, 1);
            menuLevel = 0;
            break;
        
        default:
            menuLevel = 1;
            break;
        }
    }
}
*/

void pumpLogix(void)
{

}

void ioConfig(void *pvParameters)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf1 = {};
    // set as output mode
    io_conf1.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf1.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf1.pull_down_en = 0;
    // disable pull-up mode
    io_conf1.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf1);
    gpio_set_level(OE_595, 1);
    Send_74HC595(0, 0, 0);
    gpio_set_level(OE_595, 0);
    gpio_set_level(LED, 1);

    // bit mask of the pins, use GPIO4/5 here
    io_conf1.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf1.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf1.pull_up_en = 1;
    gpio_config(&io_conf1);

    // bit mask of the pins, use GPIO4/5 here
    io_conf1.pin_bit_mask = DATA165;
    // set as input mode
    io_conf1.mode = GPIO_MODE_INPUT;
    // disable pull-up mode
    io_conf1.pull_up_en = 0;
    gpio_config(&io_conf1);
    vTaskDelete(NULL);
}

static void enterMenu_cb(void *arg)
{
    mainLogix = 1;
    emenu = 1;
}

void ctrlCenter(void *pvParameters)
{
    uint8_t msgCounter = 0;
    uint8_t switchCounter = 0;
    button_event_config_t enterMenucfg = {
        .event = BUTTON_LONG_PRESS_UP,
        .event_data.long_press.press_time = 6000,
    };
    button_handle_t gpio_btn1 = iot_button_create(&gpio_btn1_cfg);
    button_handle_t gpio_btn2 = iot_button_create(&gpio_btn2_cfg);
    button_handle_t gpio_btn3 = iot_button_create(&gpio_btn3_cfg);
    button_handle_t gpio_btn4 = iot_button_create(&gpio_btn4_cfg);
    while (mainLogix > 0)
    {
        iot_button_register_event_cb(gpio_btn1, enterMenucfg, enterMenu_cb, NULL);
        iot_button_register_event_cb(gpio_btn2, enterMenucfg, enterMenu_cb, NULL);
        iot_button_register_event_cb(gpio_btn3, enterMenucfg, enterMenu_cb, NULL);
        iot_button_register_event_cb(gpio_btn4, enterMenucfg, enterMenu_cb, NULL);
        mainLogix = 2;
        while ((mainLogix == 2) & (menuLevel == 0)) 
        {
            if (msgCounter == 50)
            {
                msgCounter = 0;
                xTaskCreate(&rotateMessages, "rotateMessages", 2048, NULL, 1, NULL);
            }
            msgCounter++;
            if (switchCounter == 40)
            {
                switchCounter = 0;
                switchSet();
            }
            switchCounter++;
            vTaskDelay(pdTICKS_TO_MS(10));
        }
        if (emenu == 1)
        {
            emenu = 0;
            menuLevel = 1;
            iot_button_unregister_event(gpio_btn1, enterMenucfg, enterMenu_cb);
            iot_button_unregister_event(gpio_btn2, enterMenucfg, enterMenu_cb);
            iot_button_unregister_event(gpio_btn3, enterMenucfg, enterMenu_cb);
            iot_button_unregister_event(gpio_btn4, enterMenucfg, enterMenu_cb);
            printf("Menu Level: %d\n", menuLevel);
            menuLevel = 0;
            printf("Reset Menu Level: %d\n", menuLevel);
            //xTaskCreate(&mainMenu, "mainMenu", 2048, NULL, 2, NULL);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    setMessage(21, 1);
    xTaskCreate(&ioConfig, "ioConfig", 2048, NULL, 1, NULL);
    xTaskCreate(&initReadNvs, "initReadNvs", 2048, NULL, 1, NULL);
    xTaskCreate(&outputSend, "outputSend", 2048, NULL, 2, NULL);
    mainLogix = 1;
    xTaskCreate(&ctrlCenter, "ctrlCenter", 8192, NULL, 1, NULL);
}