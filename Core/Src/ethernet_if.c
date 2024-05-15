#include "main.h"
#include "ethernet_if.h"

#define BUFFER_SIZE 512

extern NetworkInterface_t *pxSTM32Fxx_FillInterfaceDescriptor(BaseType_t xEMACIndex,
                                                              NetworkInterface_t *pxInterface);
static void prvCreateTCPServerSocketTasks(void *pvParameters);
static void prvServerConnectionInstance(void *pvParameters);
static void prvEchoClientRxTask(void *pvParameters);

NetworkInterface_t xInterfaces[1];
struct xNetworkEndPoint xEndPoints[1];
uint8_t socketShutdownTimeout = 0;
extern TaskHandle_t xHandleUpdatePulseData;
extern volatile uint32_t pulseFrequency;

BaseType_t tcp_server_init()
{
    /* Initialise the interface descriptor for WinPCap for example. */
    pxSTM32Fxx_FillInterfaceDescriptor(0, &(xInterfaces[0]));

    FreeRTOS_FillEndPoint(&(xInterfaces[0]), &(xEndPoints[0]), IPAddr,
                          NetMask, GatewayAddr, DNSAddr, MACAddr);
#if (ipconfigUSE_DHCP != 0)
    {
        /* End-point 0 wants to use DHCPv4. */
        xEndPoints[0].bits.bWantDHCP = pdTRUE;
    }
#endif /* ( ipconfigUSE_DHCP != 0 ) */

    /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
       are created in the vApplicationIPNetworkEventHook() hook function
       below.  The hook function is called when the network connects. */
    FreeRTOS_IPInit_Multi();
    return SUCCESS;
}

/**
 * @brief  Initializes the ETH MSP.
 * @param  ethHandle: ETH handle
 * @retval None
 */

void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (ethHandle->Instance == ETH)
    {
        /* USER CODE BEGIN ETH_MspInit 0 */

        /* USER CODE END ETH_MspInit 0 */
        /* Enable Peripheral clock */
        __HAL_RCC_ETH_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        /**ETH GPIO Configuration
        PC1     ------> ETH_MDC
        PA1     ------> ETH_REF_CLK
        PA2     ------> ETH_MDIO
        PA7     ------> ETH_CRS_DV
        PC4     ------> ETH_RXD0
        PC5     ------> ETH_RXD1
        PB13     ------> ETH_TXD1
        PG11     ------> ETH_TX_EN
        PG13     ------> ETH_TXD0
        */
        GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_TXD1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        /* ETH interrupt Init */
        HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_IRQn);
        HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);

        /* USER CODE BEGIN ETH_MspInit 1 */

        /* USER CODE END ETH_MspInit 1 */
    }
}

void vLoggingPrintf(const char *pcFormatString, ...)
{
    // SEGGER_SYSVIEW_PrintfTarget(pcFormatString);
    va_list arg;

    va_start(arg, pcFormatString);
    vprintf(pcFormatString, arg);
    va_end(arg);
}

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t eNetworkEvent,
                                          struct xNetworkEndPoint *pxEndPoint)
{
    static BaseType_t xTasksAlreadyCreated = pdFALSE;

    /* Both eNetworkUp and eNetworkDown events can be processed here. */
    if (eNetworkEvent == eNetworkUp)
    {
        /* Create the tasks that use the TCP/IP stack if they have not already
        been created. */
        if (xTasksAlreadyCreated == pdFALSE)
        {
            /*
             * For convenience, tasks that use FreeRTOS-Plus-TCP can be created here
             * to ensure they are not created before the network is usable.
             */

            xTasksAlreadyCreated = pdTRUE;

            vStartSimpleTCPServerTasks(2 * configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY + 1);
        }
    }
    /* Print out the network configuration, which may have come from a DHCP
     * server. */
    // showEndPoint(pxEndPoint);
}

/* Called by FreeRTOS-Plus-TCP */
BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
    *pulNumber = HAL_GetTick();
    return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort)
{
    uint32_t pulNumber = 0;
    xApplicationGetRandomNumber(&pulNumber);
    return pulNumber;
}

/* Stores the stack size passed into vStartSimpleTCPServerTasks() so it can be
 * reused when the server listening task creates tasks to handle connections. */
static uint16_t usUsedStackSize = 0;

void vStartSimpleTCPServerTasks(uint16_t usStackSize, UBaseType_t uxPriority)
{
    xTaskCreate(prvCreateTCPServerSocketTasks, "TCPServerListener", usStackSize, NULL, uxPriority + 1, NULL);

    /* Remember the requested stack size so it can be re-used by the server
     * listening task when it creates tasks to handle connections. */
    usUsedStackSize = usStackSize;
}

static void prvCreateTCPServerSocketTasks(void *pvParameters)
{
    struct freertos_sockaddr xClient, xBindAddress;
    Socket_t xListeningSocket, xConnectedSocket;
    socklen_t xSize = sizeof(xClient);
    static const TickType_t xReceiveTimeOut = portMAX_DELAY;
    const BaseType_t xBacklog = 10;

    /* Attempt to open the socket. */
    xListeningSocket = FreeRTOS_socket(FREERTOS_AF_INET4,    /* Or FREERTOS_AF_INET6 for IPv6. */
                                       FREERTOS_SOCK_STREAM, /* SOCK_STREAM for TCP. */
                                       FREERTOS_IPPROTO_TCP);

    /* Check the socket was created. */
    configASSERT(xListeningSocket != FREERTOS_INVALID_SOCKET);

    /* If FREERTOS_SO_RCVBUF or FREERTOS_SO_SNDBUF are to be used with
    FreeRTOS_setsockopt() to change the buffer sizes from their default then do
    it here!.  (see the FreeRTOS_setsockopt() documentation. */

    /* If ipconfigUSE_TCP_WIN is set to 1 and FREERTOS_SO_WIN_PROPERTIES is to
    be used with FreeRTOS_setsockopt() to change the sliding window size from
    its default then do it here! (see the FreeRTOS_setsockopt()
    documentation. */

    /* Set a time out so accept() will just wait for a connection. */
    FreeRTOS_setsockopt(xListeningSocket,
                        0,
                        FREERTOS_SO_RCVTIMEO,
                        &xReceiveTimeOut,
                        sizeof(xReceiveTimeOut));

    /* Set the listening port to 10000. */
    // xBindAddress.sin_address = (IP_Address_t)FreeRTOS_inet_addr_quick(0, 0, 0, 0);
    xBindAddress.sin_port = (uint16_t)LISTENING_PORT;
    xBindAddress.sin_port = FreeRTOS_htons(xBindAddress.sin_port);
    xBindAddress.sin_family = FREERTOS_AF_INET;

    /* Bind the socket to the port that the client RTOS task will send to. */
    FreeRTOS_bind(xListeningSocket, &xBindAddress, sizeof(xBindAddress));

    /* Set the socket into a listening state so it can accept connections.
    The maximum number of simultaneous connections is limited to 20. */
    FreeRTOS_listen(xListeningSocket, xBacklog);

    for (;;)
    {
        /* Wait for incoming connections. */
        xConnectedSocket = FreeRTOS_accept(xListeningSocket, &xClient, &xSize);
        configASSERT(xConnectedSocket != FREERTOS_INVALID_SOCKET);

        /* Spawn a RTOS task to handle the connection. */
        xTaskCreate(prvEchoClientRxTask,
                    "EchoServer",
                    usUsedStackSize,
                    (void *)xConnectedSocket,
                    tskIDLE_PRIORITY + 1,
                    NULL);
    }
}

void vTimeoutCallback(TimerHandle_t xTimer)
{
    socketShutdownTimeout = 1;
}

uint8_t isDigit(char c)
{
    return c >= '0' && c <= '9';
}

void prvProcessData(char *cRxedData, BaseType_t lBytesReceived, Socket_t xConnectedSocket)
{
    FreeRTOS_debug_printf(("Received data: %s\n", cRxedData));

    // Echo back the received data
    FreeRTOS_send(xConnectedSocket, cRxedData, lBytesReceived, 0);

    if (cRxedData[0] != '$')
        return;

    BaseType_t i = 1;

    while (i < lBytesReceived)
    {
        switch (cRxedData[i])
        {
        case 'f':
        case 'F':
            i++;
            pulseFrequency = 0;
            while (cRxedData[i] != '\n' && i < lBytesReceived)
            {
                if (isDigit(cRxedData[i]))
                {
                    // cRxedData[i] is an ASCII code number
                    pulseFrequency = pulseFrequency * 10 + (cRxedData[i] - '0');
                }
                i++;
            }
            FreeRTOS_debug_printf(("Pulse frequency: %d\n", pulseFrequency));
            xTaskNotifyGive(xHandleUpdatePulseData);

            break;
        default:
            i++;
            break;
        }
    }
}

static void prvEchoClientRxTask(void *pvParameters)
{
    FreeRTOS_debug_printf(("Client connected\n"));

    Socket_t xSocket;
    static char cRxedData[BUFFER_SIZE];
    BaseType_t lBytesReceived;

    /* It is assumed the socket has already been created and connected before
    being passed into this RTOS task using the RTOS task's parameter. */
    xSocket = (Socket_t)pvParameters;

    for (;;)
    {
        /* Receive another block of data into the cRxedData buffer. */
        lBytesReceived = FreeRTOS_recv(xSocket, &cRxedData, BUFFER_SIZE, 0);

        if (lBytesReceived > 0)
        {
            /* Data was received, process it here. */
            prvProcessData(cRxedData, lBytesReceived, xSocket);
        }
        else if (lBytesReceived == 0)
        {
            /* No data was received, but FreeRTOS_recv() did not return an error.
            Timeout? */
        }
        else
        {
            /* Error (maybe the connected socket already shut down the socket?).
            Attempt graceful shutdown. */
            FreeRTOS_shutdown(xSocket, FREERTOS_SHUT_RDWR);
            break;
        }
    }

    /* The RTOS task will get here if an error is received on a read.  Ensure the
    socket has shut down (indicated by FreeRTOS_recv() returning a -pdFREERTOS_ERRNO_EINVAL
    error before closing the socket). */

    // Start a timeout watchdog to avoid blocking the task indefinitely
    socketShutdownTimeout = 0;
    TimerHandle_t xTimer = xTimerCreate("SocketShutdownTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, vTimeoutCallback);

    if (xTimer == NULL)
    {
        FreeRTOS_debug_printf(("Failed to create timer\n"));
    }

    // start timer
    if (xTimerStart(xTimer, 0) != pdPASS)
    {
        FreeRTOS_debug_printf(("Failed to start timer\n"));
    }

    while (FreeRTOS_recv(xSocket, NULL, 0, 0) >= 0 && !socketShutdownTimeout)
    {
        /* Wait for shutdown to complete.  If a receive block time is used then
        this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
        into the Blocked state anyway. */
        vTaskDelay(pdMS_TO_TICKS(250));

        /* Note - real applications should implement a timeout here, not just
        loop forever. */
    }

    // reset timer
    if (xTimerReset(xTimer, 0) != pdPASS)
    {
        FreeRTOS_debug_printf(("Failed to reset timer\n"));
    }

    /* Shutdown is complete and the socket can be safely closed. */
    FreeRTOS_closesocket(xSocket);

    /* Must not drop off the end of the RTOS task - delete the RTOS task. */
    vTaskDelete(NULL);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    if ((heth->Instance->DMASR & ETH_DMA_FLAG_AIS) != 0)
    {
        /* clear interrupt flag */
        __HAL_ETH_DMA_CLEAR_FLAG(heth, ETH_DMA_FLAG_AIS);
        FreeRTOS_debug_printf(("ETH DMA Error: Abnormal interrupt summary\n"));
    }
}