/*******************************************************************************************************************//**
 * Audio driver for RZ/A3M Board
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "bsp_api.h"
#include "../inc/audio.h"
#include "../../da7212/da7212.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define DEBUG_WRITE_ONLY
//#define DEBUG_READ_ONLY
//#define DEBUG_READ_WRITE

#define AUDIO_EVENT_FLAG_I2S_IDEL   (0x00000001UL)
#define AUDIO_EVENT_FLAG_I2S_TXEMP  (0x00000002UL)
#define AUDIO_EVENT_FLAG_I2S_RXFLL  (0x00000004UL)
#define AUDIO_EVENT_FLAG_I2S_ERR    (0x00000008UL)
#define AUDIO_EVENT_FLAG_I2S_MASK   (AUDIO_EVENT_FLAG_I2S_IDEL|AUDIO_EVENT_FLAG_I2S_TXEMP|AUDIO_EVENT_FLAG_I2S_RXFLL|AUDIO_EVENT_FLAG_I2S_ERR)
#define AUDIO_I2S_WAIT              (10000)
#define AUDIO_BUF_NUM               2
//#define WAV_SIZE                    1920044
#define WAV_SIZE                    2078252
//#define WAV_TRANSFER_DATA           (960000)
#define WAV_TRANSFER_DATA           (1039104)

// WAV DATA SIZE
#define WAV_CHUNKID_SIZE            (4)
#define WAV_CHUNK_DATA_SIZE         (4)
#define WAV_FORMAT_SIZE             (4)
#define WAV_SUBCHUNKID1_SIZE        (4)
#define WAV_SUBCHUNKID1_DATA_SIZE   (4)
#define WAV_AUDIO_FORMAT_SIZE       (2)
#define WAV_CHANNEL_SIZE            (2)
#define WAV_SAMPLINGRATE_SIZE       (4)
#define WAV_AVEBYTEPERSEC_SIZE      (4)
#define WAV_BLOCKSIZE_SIZE          (2)
#define WAV_BITPERSAMPLE_SIZE       (2)
#define WAV_SUBCHUNKID2_SIZE        (4)
#define WAV_SUBCHUNKID2_DATA_SIZE   (4)
// WAV DATA OFFSET
#define WAV_CHUNK_DATA_OFST         (4)
#define WAV_FORMAT_OFST             (8)
#define WAV_SUBCHUNKID1_OFST        (12)
#define WAV_SUBCHUNKID1_DATA_OFST   (16)
#define WAV_AUDIO_FORMAT_OFST       (20)
#define WAV_CHANNEL_OFST            (22)
#define WAV_SAMPLINGRATE_OFST       (24)
#define WAV_AVEBYTEPERSEC_OFST      (28)
#define WAV_BLOCKSIZE_OFST          (32)
#define WAV_BITPERSAMPLE_OFST       (34)
#define WAV_SUBCHUNKID2_OFST        (40)
#define WAV_SUBCHUNKID2_DATA_OFST   (44)


/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/
static TX_EVENT_FLAGS_GROUP event_flag;

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/


/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void audio_i2s_open( void );
static void audio_i2s_callback( i2s_callback_args_t *p_args );
#if defined(DEBUG_WRITE_ONLY) | defined(DEBUG_READ_WRITE)
static void audio_i2s_write ( void );
#endif
#if defined(DEBUG_READ_ONLY) | defined(DEBUG_READ_WRITE)
static void audio_i2s_read ( void );
#endif
#ifdef DEBUG_READ_WRITE
static void audio_i2s_stop ( void );
#endif
static void audio_i2s_close ( void );
static UINT audio_i2s_wait_cb ( i2s_event_t *p_event );
static void audio_start( void );
static void audio_calculate_sample(uint32_t buffer_index);


/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
extern const uint8_t g_wavdata[];

static i2s_instance_t *p_i2s_instance;
#if defined(DEBUG_WRITE_ONLY)
static unsigned char audio_data[50000000];
static audio_info_t ad_info = {0};
static uint32_t g_buff_index = 0;
static int16_t stream_src[AUDIO_BUF_NUM][WAV_TRANSFER_DATA];
static bool g_data_ready = false;
static bool g_send_data_in_main_loop = true;
#endif
#if defined(DEBUG_READ_ONLY) | defined(DEBUG_READ_WRITE)
static int16_t read_data_buffer[AUDIO_BUF_NUM][WAV_TRANSFER_DATA*2];
static uint32_t g_read_buff_index = 0;
static ad_state_t g_ad_state = AD_STATE_READ_START;
static int16_t write_data_buffer[AUDIO_BUF_NUM][WAV_TRANSFER_DATA*2];
#endif

/*******************************************************************************************************************//**
 * @addtogroup AUDIO
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/
void audio_ctrl_func( i2c_master_instance_t *p_i2c, i2s_instance_t *p_i2s )
{
    /* DA7212 initialize */
    da7212_init(p_i2c);

    p_i2s_instance = p_i2s;

    /* i2s open */
    audio_i2s_open();

    /* audio start */
    //audio_start();
}


/*******************************************************************************************************************//**
 * @} (end addtogroup AUDIO)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
static void audio_i2s_open( void )
{
    fsp_err_t err;

    /* open i2s */
    err = p_i2s_instance->p_api->open(p_i2s_instance->p_ctrl, p_i2s_instance->p_cfg);
    if( FSP_SUCCESS != err )
    {
        printf("[AUDIO] I2S open failed.(%d)\r\n", err);
        return;
    }

    /* set callback */
    err = p_i2s_instance->p_api->callbackSet(p_i2s_instance->p_ctrl, audio_i2s_callback, NULL, NULL);
    if( FSP_SUCCESS != err )
    {
        printf("[AUDIO] I2S callbackSet failed.(%d)\r\n", err);
        return;
    }
}

static void audio_i2s_close( void )
{
    fsp_err_t err;

    // I2S Close
    err = p_i2s_instance->p_api->close(p_i2s_instance->p_ctrl);
    if(FSP_SUCCESS != err)
    {
        printf("[AUDIO] I2S close failed.(%d)\r\n", err);
    }
}

static void audio_i2s_callback( i2s_callback_args_t *p_args )
{
    //FSP_PARAMETER_NOT_USED(p_args);
    switch(p_args->event)
    {
#ifdef DEBUG_WRITE_ONLY
    case I2S_EVENT_IDLE:
    case I2S_EVENT_TX_EMPTY:
        if (g_data_ready)
        {
            audio_i2s_write();
        }
        else
        {
            g_send_data_in_main_loop = true;
        }
        break;
    case I2S_EVENT_RX_FULL:
    default:
        break;
#endif
#ifdef DEBUG_READ_ONLY
    case I2S_EVENT_IDLE:
    case I2S_EVENT_RX_FULL:
    	g_read_data_ready = true;
        break;
    case I2S_EVENT_TX_EMPTY:
    default:
        break;
#endif
#ifdef DEBUG_READ_WRITE
    case I2S_EVENT_IDLE:
        if(g_ad_state == AD_STATE_READ_END)
        {
            g_ad_state = AD_STATE_WRITE_START;
        }
        else if(g_ad_state == AD_STATE_WRITE_END)
        {
            g_ad_state = AD_STATE_READ_START;
        }
        break;
    case I2S_EVENT_RX_FULL:
        g_ad_state = AD_STATE_READ_END;
        audio_i2s_stop();
        break;
    case I2S_EVENT_TX_EMPTY:
        g_ad_state = AD_STATE_WRITE_END;
        audio_i2s_stop();
        break;
    default:
        break;
#endif
    }
}

#if defined(DEBUG_WRITE_ONLY) | defined(DEBUG_READ_WRITE)
static void audio_i2s_write (void)
{
    fsp_err_t err = FSP_SUCCESS;
 #ifdef DEBUG_WRITE_ONLY
    err = p_i2s_instance->p_api->write( p_i2s_instance->p_ctrl,
                                        (uint8_t *) &stream_src[g_buff_index][0],
                                        ad_info.subckSize_2);
 #endif
 #ifdef DEBUG_READ_WRITE
    err = p_i2s_instance->p_api->write( p_i2s_instance->p_ctrl,
                                        (uint8_t *) &write_data_buffer[g_read_buff_index][0],
                                        WAV_TRANSFER_DATA*2
                                      );
 #endif
    if (FSP_SUCCESS != err)
    {
        printf("[AUDIO] I2S write failed.(%d)\r\n", err);
        return;
    }
    else
    {
 #ifdef DEBUG_WRITE_ONLY
    	g_buff_index = !g_buff_index;
        g_data_ready = false;
 #endif
 #ifdef DEBUG_READ_WRITE
        printf("write\r\n");
        g_ad_state = AD_STATE_STAY;
        g_read_buff_index = !g_read_buff_index;
 #endif
    }
}
#endif
#if defined(DEBUG_READ_ONLY) | defined(DEBUG_READ_WRITE)
static void audio_i2s_read (void)
{
    fsp_err_t err;
    err = p_i2s_instance->p_api->read( p_i2s_instance->p_ctrl,
                                       (uint8_t *) &read_data_buffer[g_read_buff_index][0],
                                       WAV_TRANSFER_DATA*2
                                     );
    if (FSP_SUCCESS != err)
    {
        printf("[AUDIO] I2S read failed.(%d)\r\n", err);
        return;
    }
    else
    {
 #ifdef DEBUG_READ_ONLY
        g_read_data_ready = false;
 #endif
 #ifdef DEBUG_READ_WRITE
        printf("read\r\n");
        g_ad_state = AD_STATE_STAY;
 #endif
    }
}
#endif
#if defined(DEBUG_WRITE_ONLY) | defined(DEBUG_READ_WRITE)
static void audio_calculate_sample(uint32_t buffer_index)
{
    uint32_t i;
 #ifdef DEBUG_WRITE_ONLY
    for (i = 0; i < (ad_info.subckSize_2 / 4); i++)
    {
        stream_src[buffer_index][2*i] = (int16_t)( (int16_t)(audio_data[WAV_SUBCHUNKID2_DATA_OFST + (4*i)])
                                                 |((int16_t)audio_data[WAV_SUBCHUNKID2_DATA_OFST + (4*i + 1)]) << 8
                                                 );
        stream_src[buffer_index][2*i + 1] = (int16_t)( (int16_t)(audio_data[WAV_SUBCHUNKID2_DATA_OFST + (4*i + 2)])
                                                     |((int16_t)audio_data[WAV_SUBCHUNKID2_DATA_OFST + (4*i + 3)]) << 8
                                                     );
    }
    /* Data is ready to be sent in the interrupt. */
    g_data_ready = true;
 #endif /* DEBUG_WRITE_ONLY */
 #ifdef DEBUG_READ_WRITE
    for (i = 0; i < WAV_TRANSFER_DATA*2; i++)
    {
        write_data_buffer[buffer_index][i] = read_data_buffer[g_read_buff_index][i];
    }
 #endif
}
#endif
#if defined(DEBUG_READ_WRITE)
static void audio_i2s_stop(void)
{
    fsp_err_t err;
    err = p_i2s_instance->p_api->stop(p_i2s_instance->p_ctrl);

    if (FSP_SUCCESS != err)
    {
        //printf("[AUDIO] I2S stop failed.(%d)\r\n", err);
        return;
    }
}
#endif
static void audio_start(void)
{
#ifdef DEBUG_WRITE_ONLY
    int i;
    // read audio data
    memcpy(audio_data, g_wavdata, WAV_SIZE );

    // Retrieving audio information
    // chunk size
    for (i=0; i < WAV_CHUNK_DATA_SIZE; i++)
    {
        ad_info.ckSize |= ((uint32_t)(audio_data[WAV_CHUNK_DATA_OFST + i]) << (i*8));
    }
    printf("[AUDIO] Chunk size. (%#x)\r\n", ad_info.ckSize);

    // sub chunk size
    for (i=0; i < WAV_SUBCHUNKID1_DATA_SIZE; i++)
    {
        ad_info.subckSize_1 |= ((uint32_t)(audio_data[WAV_SUBCHUNKID1_DATA_OFST + i]) << (i*8));
    }
    printf("[AUDIO] Sub Chunk size. (%#x)\r\n", ad_info.subckSize_1);

    // audio format
    for (i=0; i < WAV_AUDIO_FORMAT_SIZE; i++)
    {
        ad_info.adFormat |= (uint16_t)((uint16_t)(audio_data[WAV_AUDIO_FORMAT_OFST + i]) << (i*8));
    }
    printf("[AUDIO] Sub Chunk size. (%#x)\r\n", ad_info.adFormat);

    // channel
    for (i=0; i < WAV_CHANNEL_SIZE; i++)
    {
        ad_info.channel |= (uint16_t)((uint16_t)(audio_data[WAV_CHANNEL_OFST + i]) << (i*8));
    }
    printf("[AUDIO] Channel number. (%#x)\r\n", ad_info.channel);

    // sampling rate
    for (i=0; i < WAV_SAMPLINGRATE_SIZE; i++)
    {
        ad_info.sr |= ((uint32_t)(audio_data[WAV_SAMPLINGRATE_OFST + i]) << (i*8));
    }
    printf("[AUDIO] sampling rate. (%d)\r\n", ad_info.sr);

    // average bytes per second
    for (i=0; i < WAV_AVEBYTEPERSEC_SIZE; i++)
    {
        ad_info.avgBytesPerSec |= ((uint32_t)(audio_data[WAV_AVEBYTEPERSEC_OFST + i]) << (i*8));
    }
    printf("[AUDIO] average bytes per second. (%d)\r\n", ad_info.avgBytesPerSec);

    // block size
    for (i=0; i < WAV_BLOCKSIZE_SIZE; i++)
    {
        ad_info.blockAlign |= (uint16_t)((uint16_t)(audio_data[WAV_BLOCKSIZE_OFST + i]) << (i*8));
    }
    printf("[AUDIO] block size. (%d)\r\n", ad_info.blockAlign);

    // bit per sample
    for (i=0; i < WAV_BITPERSAMPLE_SIZE; i++)
    {
        ad_info.bitsPerSample |= (uint16_t)((uint16_t)(audio_data[WAV_BITPERSAMPLE_OFST + i]) << (i*8));
    }
    printf("[AUDIO] bit per sample number. (%d)\r\n", ad_info.bitsPerSample);

    // audio data size
    for (i=0; i < WAV_SUBCHUNKID2_DATA_SIZE; i++)
    {
        ad_info.subckSize_2 |= ((uint32_t)(audio_data[WAV_SUBCHUNKID2_OFST + i]) << (i*8));
    }
    printf("[AUDIO] Data size. (%#x)\r\n", ad_info.subckSize_2);
#endif

    while(true)
    {
#ifdef DEBUG_WRITE_ONLY
        // audio data set
        audio_calculate_sample(g_buff_index);
        if(g_send_data_in_main_loop)
        {
            /* Clear flag. */
            g_send_data_in_main_loop = false;
            /* Reload FIFO and handle errors. */
            audio_i2s_write();
        }
        while(g_data_ready)
        {
            // Do nothing
        }
#endif /* DEBUG_READ_WRITE */
#ifdef DEBUG_READ_ONLY
        // Read data
        audio_i2s_read();
        printf("g_read_data_ready is %d \r\n", g_read_data_ready);
        while(!g_read_data_ready)
        {
            /* Do nothing. */
        }
        g_read_buff_index = !g_read_buff_index;
#endif /* DEBUG_READ_ONLY */
#ifdef DEBUG_READ_WRITE
        if(g_ad_state == AD_STATE_READ_START)
        {
            // Read data
            audio_i2s_read();
        }
        else if(g_ad_state == AD_STATE_WRITE_START)
        {
            // Write data
            audio_calculate_sample(g_read_buff_index);
            audio_i2s_write();
        }
        else
        {
            // Do nothing
        }
        while(!(g_ad_state == AD_STATE_READ_START || g_ad_state == AD_STATE_WRITE_START))
        {
            // Do nothing
        }
#endif
    }
}

static UINT audio_i2s_wait_cb ( i2s_event_t *p_event )
{
    UINT status;
    ULONG statusbit;

    status = tx_event_flags_get( &event_flag, AUDIO_EVENT_FLAG_I2S_MASK, TX_OR_CLEAR, &statusbit, AUDIO_I2S_WAIT );

    if( TX_SUCCESS == status )
    {
        if( (statusbit & AUDIO_EVENT_FLAG_I2S_IDEL) == AUDIO_EVENT_FLAG_I2S_IDEL )
        {
            *p_event = I2S_EVENT_IDLE;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2S_TXEMP) == AUDIO_EVENT_FLAG_I2S_TXEMP )
        {
            *p_event = I2S_EVENT_TX_EMPTY;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2S_RXFLL) == AUDIO_EVENT_FLAG_I2S_RXFLL )
        {
            *p_event = I2S_EVENT_RX_FULL;
        }
        else if( (statusbit & AUDIO_EVENT_FLAG_I2S_ERR) == AUDIO_EVENT_FLAG_I2S_ERR )
        {
            status = TX_NOT_AVAILABLE;
        }
        else
        {
            status = TX_NOT_AVAILABLE;
        }
        return status;
    }
    else
    {
        printf("[AUDIO] tx_event_flags_get failed(%X)\r\n", status);
        return TX_WAIT_ERROR;
    }
}
