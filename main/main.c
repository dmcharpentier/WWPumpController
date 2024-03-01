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

/*
#define KEY1 18
#define KEY2 19
#define KEY3 21
#define KEY4 23
*/
#define KEY1 GPIO_NUM_18
#define KEY2 GPIO_NUM_19
#define KEY3 GPIO_NUM_21
#define KEY4 GPIO_NUM_23
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
uint8_t Dot = 0x80;
uint8_t BitsSele = 0;

// Loop Logix Variables
int counter = 0; // Initialize counter

// LSB (least significant bit) first or MSB (most significant bit) first
#define LSBFIRST 0
#define MSBFIRST 1

// Message Variables
#define MAX_MSGS 10                 // Adjust as needed
#define MSG_SIZE 5                  // Number of segments per message
#define SEGMENT_CODES 29            // Number of entries in SEG8Code
uint8_t activeMsg[MAX_MSGS][MSG_SIZE];
uint8_t curDisplay[MSG_SIZE];
int msgCount = 0;
int currentIndex = 0;
int syncCounter = 0;          // Counter to sync messages
uint32_t activeMessagesMask = 0;  // Tracks messages currently active
uint32_t messagesInArrayMask = 0; // Tracks messages currently in the array

// Menu Variables
#define BUTTON1_PRESS 1
#define BUTTON2_PRESS 2
#define BUTTON3_PRESS 3
#define BUTTON4_PRESS 4
QueueHandle_t buttonEventQueue = NULL;
volatile int menuLevel = 0;
volatile int temp = 0;
int alternatePump = 0;


// NVS Variables
nvs_handle_t settings;
int32_t settingVar = 0; // value will default to 0, if not set yet in NVS

uint8_t SEG8Code[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00}; // Common anode Digital Tube Character Gallery

// Logix Variables
uint16_t Time_Cnt = 0;
uint8_t Out_Value = 0;
uint8_t Out_Cnt = 0;
uint8_t Value = 0;
uint8_t Relays = 0;

// ISR handler for button presses
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int buttonNum = (int)arg;
    int event = 0;
    switch (buttonNum) {
        case KEY1:
            event = BUTTON1_PRESS;
            break;
        case KEY2:
            event = BUTTON2_PRESS;
            break;
        case KEY3:
            event = BUTTON3_PRESS;
            break;
        case KEY4:
            event = BUTTON4_PRESS;
            break;
        // Add cases for other buttons
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(buttonEventQueue, &event, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(); // Ensure higher priority tasks are executed immediately
    }
}

// Initialize GPIO and ISR for button handling
void gpio_init(void) {
    gpio_config_t io_conf;
    // Zeroing the config structure
    memset(&io_conf, 0, sizeof(gpio_config_t));

    // Configure GPIOs for input, pull-up, and interrupt on falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<KEY1) | (1ULL<<KEY2) | (1ULL<<KEY3) | (1ULL<<KEY4); // Repeat for other buttons
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Initialize the queue
    buttonEventQueue = xQueueCreate(10, sizeof(int));

    // Install ISR service and add handlers for each button
    gpio_install_isr_service(0);
    gpio_isr_handler_add(KEY1, gpio_isr_handler, (void*)KEY1);
    gpio_isr_handler_add(KEY2, gpio_isr_handler, (void*)KEY2);
    gpio_isr_handler_add(KEY3, gpio_isr_handler, (void*)KEY3);
    gpio_isr_handler_add(KEY4, gpio_isr_handler, (void*)KEY4);
    // Repeat for other buttons
}

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

void initReadNvs(void *pvParameters)
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
    vTaskDelete(NULL);
}

void setNvs(void)
{
    esp_err_t err = nvs_flash_init();
    // Write
    err = nvs_open("storage", NVS_READWRITE, &settings);
    err = nvs_set_i32(settings, "settings", settingVar);

    // Commit written value.
    err = nvs_commit(settings);

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

/*********************************************************
 *
 *                     Messaging System
 *
 ********************************************************/
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

// Compares two messages for equality
bool areMessagesEqual(uint8_t msg1[], uint8_t msg2[]) {
    for (int i = 0; i < MSG_SIZE; i++) {
        if (msg1[i] != msg2[i]) {
            return false;
        }
    }
    return true;
}

void setMessage (int option, bool value) {
    if (value) {
        activeMessagesMask |= 1 << option;
    } else {
        activeMessagesMask &= ~(1 << option);
    }
}

// Function to add a message to activeMsg if it's not already added
void addMsg(int option) {
    if (!(messagesInArrayMask & 1 << option)) { // Check if message is not already active
        if (msgCount < MAX_MSGS) {
            for (int i = 0; i < MSG_SIZE; i++)
            {
                activeMsg[msgCount][i] = statusMessages[option][i];
            }
            msgCount++;
            messagesInArrayMask |= 1 << option; // Mark as active
        }
    }
}

void deleteMsg(int option) {
    if (messagesInArrayMask & 1 << option) {
        // Find and remove the message from activeMsg
        for (int i = 0; i < msgCount; i++) {
            if (areMessagesEqual(activeMsg[i], statusMessages[option])) {
                for (int j = i; j < msgCount - 1; j++) {
                    for (size_t r = 0; i < MSG_SIZE; i++)
                    {
                        activeMsg[j][r] = activeMsg[j + 1][r]; // Shift messages down
                    }
                }
                msgCount--;
                messagesInArrayMask &= ~(1 << option); // Clear from mask
                break;
            }
        }
    }
}

// Synchronizes the active messages with the display array
void syncActiveMessages(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(250);
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t currentMask = activeMessagesMask;
        for (int option = 0; option < 32; option++)
        {
            uint32_t mask = 1 << option;
            if ((currentMask & mask) && !(messagesInArrayMask & mask))
            {
                addMsg(option);
            }
            else if (!(currentMask & mask) && (messagesInArrayMask & mask))
            {
                deleteMsg(option);
            }
        }
        // Delay for a period.

    }
}

/**
 * @brief Rotates the messages displayed on the screen.
 * 
 * This function updates the current message index and copies the corresponding message
 * to the curDisplay array. The messages are rotated in a circular manner.
 */
void rotateMessages()
{
  currentIndex = (currentIndex + 1) % msgCount;
  memcpy(curDisplay, activeMsg[currentIndex], sizeof(curDisplay));
}

/*********          Message System End          *********/

void outputSend(void *pvParameters)
{
    const TickType_t xDelay250ms = pdMS_TO_TICKS(5);
    while (1)
    {
        uint8_t Dot = 0x80; // Dot for display
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
        Send_74HC595(tempPx[BitsSele], BitsSelection[BitsSele], Out_Value);
        BitsSele = (BitsSele + 1) % 4;
        vTaskDelay(xDelay250ms);
    }
}

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

void initGPIO(void *pvParameters)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      //ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    gpio_init(); // Initialize button handling

    vTaskDelete(NULL);
}

static bool IRAM_ATTR sync_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    //syncActiveMessages();
    if (counter == 2)
    {
        counter = 0;
        rotateMessages();
    }
    counter++;
    return pdTRUE;
}

void syncTimerInit(void *pvParameters)
{
    gptimer_handle_t gptimer1 = NULL; // Universal Timer Handle
    gptimer_config_t timer_config = {
        // Initialization parameter setting
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select Clock Source
        .direction = GPTIMER_COUNT_UP,      // Upward Counting
        .resolution_hz = 1 * 1000 * 1000,   // 1MHz, 1 tick = 1us  Setting the Timing Time
    };
    ESP_LOGI(TAG, "Start timer, stop it at alarm event");
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer1)); // Creates a general-purpose timer that returns a task handle.

    gptimer_event_callbacks_t cbs = {
        // Interrupt callback function (alrm interrupt)
        .on_alarm = sync_timer_cb,
    };

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer1, &cbs, NULL)); // The first call to this function needs to be preceded by a call to gptimer_enable
    ESP_ERROR_CHECK(gptimer_enable(gptimer1));                               // Enable Timer Interrupt

    ESP_LOGI(TAG, "Start timer, auto-reload at alarm event");
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 250000, // period = 250ms
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer1, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer1));
    vTaskDelete(NULL);
}

static bool IRAM_ATTR io_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    switchSet();
    //outputSend();
    return pdTRUE;
}

void outputTimerInit(void *pvParameters)
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
    vTaskDelete(NULL);
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

void ctrlCenter(void)
{

}

void app_main(void)
{
    int cntr = 0;
    xTaskCreate(&ioConfig, "ioConfig", 2048, NULL, 1, NULL);
    xTaskCreate(&outputTimerInit, "outputTimerInit", 2048, NULL, 1, NULL);
    xTaskCreate(&initReadNvs, "initReadNvs", 2048, NULL, 1, NULL);
    xTaskCreate(&initGPIO, "initGPIO", 2048, NULL, 1, NULL);
    xTaskCreate(&syncTimerInit, "syncTimerInit", 2048, NULL, 1, NULL);
    xTaskCreate(&syncActiveMessages, "syncActiveMessages", 2048, NULL, 1, NULL);
    xTaskCreate(&outputSend, "outputSend", 2048, NULL, 1, NULL);
    //outputTimerInit();
    //initReadNvs();
    setMessage(21,1);
    mainMenu();
    ctrlCenter();
}