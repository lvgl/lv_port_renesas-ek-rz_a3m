/*******************************************************************************************************************//**
 * Touch panel driver for EXC80H60
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#include "tx_api.h"
#include "../inc/tp_exc80h60.h"
#include "../../i2c/inc/i2c_ctrl.h"
#ifdef TP_USE_EXC80H60
/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define TP_DEBUG
#define TP_INT_IDLE_STATE       (BSP_IO_LEVEL_HIGH)     /* no interrup pin state */

#define EXC80H_I2C_SLV_ADDR     (0x2A)      /* exc80h60 slave address */
#define EXC80H_I2C_WAIT         (500)       /* I2C Callback wait time */
#define EXC80H_I2C_MAX_LEN      (66)        /* I2C read data size max 66byte */
#define EXC80H_XY_RESO4K        (4096)      /* EXC80H X,Y max 4k resolution */
#define EXC80H_XY_RESO16K       (16384)     /* EXC80H X,Y max 16k resolution */

/***********************************************************************************************************************
 * Private constants
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
typedef struct _contact10 {
    uint8_t stat;
    uint8_t contact_id;
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t reserved[4];
} tp_10b_contact_t;

typedef struct _contact14 {
    uint8_t stat;
    uint8_t contact_id;
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t center_x_lsb;
    uint8_t center_x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t center_y_lsb;
    uint8_t center_y_msb;
    uint8_t width;
    uint8_t heght;
    uint8_t reserved[2];
} tp_14b_contact_t;

typedef struct _actpan12 {
    uint8_t stat;
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t z_lsb;
    uint8_t z_msb;
    uint8_t x_tilt_lsb;
    uint8_t x_tilt_msb;
    uint8_t y_tilt_lsb;
    uint8_t y_tilt_msb;
} tp_12b_activepen_t;

typedef struct _tp_contact {
    exc80h60_report_id_t report_id;
    int8_t rem_of_contacts;
    int8_t store_contacts;
    union {
        tp_10b_contact_t report_info_10[10];
        tp_14b_contact_t report_info_14[10];
        tp_12b_activepen_t reportinfo_apen;
    };
} tp_contact_info_t;

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void exc80h60_i2c_start( void );
static void exc80h60_irq_start( void );
static void exc80h60_i2c_stop( void );
static void exc80h60_irq_stop( void );
static void exc80h60_irq( void );
//static void exc80h60_irq( bsp_io_level_t pin_state );
static void exc80h60_i2c_callback( i2c_master_callback_args_t *p_arg );
static void exc80h60_irq_callback( external_irq_callback_args_t *p_arg );

static bool _exc80h60_i2c_scan( uint8_t *p_out );
static UINT _exc80h60_i2c_wait_cb( i2c_master_event_t *p_event );
static int8_t _exc80h60_analyze_report( uint8_t *p_in, tp_contact_info_t *p_contact );
static bool _exc80h60_analyze_touch( tp_contact_info_t *p_contact, tp_messege_info_t *p_out );

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static tp_device_init_t	*p_param = NULL;
static tp_contact_info_t _contact;
static i2c_master_instance_t *p_i2c;

/*******************************************************************************************************************//**
 * @addtogroup TP_EXC80H60
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/**
 * @brief tp_exc80h60_thread_entry
 * @param thread_input
 */
void tp_exc80h60_thread_entry(ULONG thread_input)
{
	FSP_PARAMETER_NOT_USED(thread_input);
	UINT status;
	ULONG statusbit;
	bsp_io_level_t pin_value;
	uint8_t tp_info[EXC80H_I2C_MAX_LEN] = {0};

	printf("[EXC80] tp_exc80h60_thread_entry start.\r\n");

	while(true)
	{

#if 0
        g_ioport.p_api->pinRead( g_ioport.p_ctrl, TP_INT_PORT, &pin_value );
        if( pin_value == BSP_IO_LEVEL_LOW )
        {
            printf("[EXC80] BSP_IO_LEVEL_LOW(%d)\r\n", pin_value);
        }
        else
        {
            printf("[EXC80] BSP_IO_LEVEL_HIGH(%d)\r\n", pin_value);
        }
        tx_thread_sleep(1000);

#else
		status = tx_event_flags_get( p_param->p_flag, TP_EVENT_FLAG_MASK, TX_OR, &statusbit, TX_WAIT_FOREVER );
		if( TX_SUCCESS == status )
		{
			if( (statusbit & TP_EVENT_FLAG_START) == TP_EVENT_FLAG_START )
			{
				g_ioport.p_api->pinRead( g_ioport.p_ctrl, TP_INT_PORT, &pin_value );
				exc80h60_irq_start();
                // Check if touch information is retained
                if(BSP_IO_LEVEL_LOW == pin_value)
                {
                    do
                    {
                        memset(tp_info,0x00,sizeof(tp_info));
                        if( _exc80h60_i2c_scan( tp_info ) != true )
                        {
                            printf("[EXC80] _exc80h60_i2c_scan failed.\r\n");
                            return;
                        }
                        g_ioport.p_api->pinRead( g_ioport.p_ctrl, TP_INT_PORT, &pin_value );
                    }while(BSP_IO_LEVEL_LOW == pin_value);
                }
                printf("[EXC80] TP_EVENT_FLAG_START(%d)\r\n", pin_value);
				tx_event_flags_set( p_param->p_flag, (ULONG)~TP_EVENT_FLAG_START, TX_AND );
			}
			if( (statusbit & TP_EVENT_FLAG_STOP) == TP_EVENT_FLAG_STOP )
			{
				printf("[EXC80] TP_EVENT_FLAG_STOP\r\n");
				exc80h60_irq_stop();
				tx_event_flags_set( p_param->p_flag, (ULONG)~TP_EVENT_FLAG_STOP, TX_AND );
			}
			if( (statusbit & TP_EVENT_FLAG_IRQ) == TP_EVENT_FLAG_IRQ )
			{
                exc80h60_irq();
                tx_event_flags_set( p_param->p_flag, (ULONG)~TP_EVENT_FLAG_IRQ, TX_AND );
			}
		}
		else
		{
			printf("[EXC80] tx_event_flags_get failed.(%d)\r\n", status);
			tx_thread_sleep(1000);
		}
#endif
	}
}

/**
 * @brief tp_exc80h60_init
 * @param p_init
 */
void tp_exc80h60_init( tp_device_init_t *p_init )
{
	p_param = p_init;
}

/*******************************************************************************************************************//**
 * @} (end addtogroup TP_EXC80H60)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static void exc80h60_i2c_start( void )
{
	fsp_err_t err;

	// Get i2c instance
	p_i2c = i2c_get_ch1_func();

	/* i2C HAL Driver init */
	err = p_i2c->p_api->open( p_i2c->p_ctrl, p_i2c->p_cfg );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] I2C open failed.(%d)\r\n", err);
    }
    err = p_i2c->p_api->callbackSet( p_i2c->p_ctrl, exc80h60_i2c_callback, NULL ,NULL );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] I2C callbackSet failed.(%d)\r\n", err);
    }
    err = p_i2c->p_api->slaveAddressSet( p_i2c->p_ctrl, EXC80H_I2C_SLV_ADDR, I2C_MASTER_ADDR_MODE_7BIT );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] I2C slaveAddressSet failed.(%d)\r\n", err);
    }
}

static void exc80h60_irq_start( void )
{
    fsp_err_t err;

    /* IRQ HAL Driver init */
    err = p_param->p_irq_instance->p_api->open( p_param->p_irq_instance->p_ctrl, p_param->p_irq_instance->p_cfg );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] IRQ open failed.(%d)\r\n", err);
    }
    err = p_param->p_irq_instance->p_api->callbackSet( p_param->p_irq_instance->p_ctrl, exc80h60_irq_callback, NULL, NULL );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] IRQ callbackSet failed.(%d)\r\n", err);
    }
    err = p_param->p_irq_instance->p_api->enable( p_param->p_irq_instance->p_ctrl );
    if( FSP_SUCCESS != err )
    {
        printf("[EXC80] IRQ enable failed.(%d)\r\n", err);
    }
}

static void exc80h60_i2c_stop( void )
{
	/* I2C Close */
	p_i2c->p_api->close( p_i2c->p_ctrl );
}

static void exc80h60_irq_stop( void )
{
    /* IRQ Close */
    p_param->p_irq_instance->p_api->disable( p_param->p_irq_instance->p_ctrl );
    p_param->p_irq_instance->p_api->close( p_param->p_irq_instance->p_ctrl );
}

static void exc80h60_irq( void )
{
    bsp_io_level_t pin_state;
	UINT status;
	int8_t ret;
	tp_messege_info_t *p_message;
	uint8_t tp_info[EXC80H_I2C_MAX_LEN] = {0};

	while(1)
	{
        /* touch info scan */
	    memset(tp_info,0x00,sizeof(tp_info));
        if( _exc80h60_i2c_scan( tp_info ) != true )
        {
            printf("[EXC80] _exc80h60_i2c_scan failed.\r\n");
            return;
        }
#if 0
        /* dump i2c data */
        else
        {
            int i;
            for(i=0;i<sizeof(tp_info);i++)
            {
                printf("%02X ", tp_info[i]);
            }
            printf("\r\n");
        }
#endif
        /* analysis touch info */
        ret = _exc80h60_analyze_report( tp_info, &_contact );
        if( ret == -1 )
        {
            printf("[EXC80] _exc80h60_analyze_report failed.\r\n");
            return;
        }
        else if ( ret > 0 )
        {
            /* message block pool allocate */
            status = tx_block_allocate( p_param->p_pool, (void**)&p_message, TX_NO_WAIT );
            if( TX_SUCCESS != status )
            {
                printf("[EXC80] tx_block_allocate failed(%d)\r\n",status);
                continue;
            }

            /* make message form tp data */
            _exc80h60_analyze_touch( &_contact, p_message );

#ifdef TP_DEBUG
            printf("@msg:%d,%d,%d\r\n", p_message->state,  p_message->pos_x, p_message->pos_y );
#endif
            status = tx_queue_send( p_param->p_queue, (void*)&p_message, TX_WAIT_FOREVER );
            if( TX_SUCCESS != status )
            {
                printf("[EXC80] tx_queue_send failed(%d)\r\n",status);
            }

            g_ioport.p_api->pinRead( g_ioport.p_ctrl, TP_INT_PORT, &pin_state );
            if(TP_INT_IDLE_STATE == pin_state)
            {
                break;
            }
        }
	}
}

/**
 * @brief exc80h60_irq_callback
 * @param p_arg
 */
static void exc80h60_irq_callback( external_irq_callback_args_t *p_arg )
{
	FSP_PARAMETER_NOT_USED(p_arg);
	tx_event_flags_set( p_param->p_flag, (ULONG)TP_EVENT_FLAG_IRQ, TX_OR );
}

/**
 * @brief exc80h60_i2c_callback
 * @param p_arg
 */
static void exc80h60_i2c_callback( i2c_master_callback_args_t *p_arg )
{
    FSP_PARAMETER_NOT_USED(p_arg);

    switch(p_arg->event)
    {
    case I2C_MASTER_EVENT_ABORTED:
        tx_event_flags_set( p_param->p_flag, (ULONG)TP_EVENT_FLAG_I2C_ABT, TX_OR );
        break;
    case I2C_MASTER_EVENT_RX_COMPLETE:
        tx_event_flags_set( p_param->p_flag, (ULONG)TP_EVENT_FLAG_I2C_RXC, TX_OR );
        break;
    case I2C_MASTER_EVENT_TX_COMPLETE:
        tx_event_flags_set( p_param->p_flag, (ULONG)TP_EVENT_FLAG_I2C_TXC, TX_OR );
        break;
    default:
        tx_event_flags_set( p_param->p_flag, (ULONG)TP_EVENT_FLAG_I2C_ERR, TX_OR );
        break;
    }
}

/**
 * @brief _exc80h60_i2c_wait_cb
 * @param p_event
 * @return
 */
static UINT _exc80h60_i2c_wait_cb( i2c_master_event_t *p_event )
{
    UINT status;
    ULONG statusbit;

    status = tx_event_flags_get( p_param->p_flag, TP_EVENT_FLAG_I2C_MSK, TX_OR_CLEAR, &statusbit, EXC80H_I2C_WAIT );
    if( TX_SUCCESS == status )
    {
        if( (statusbit & TP_EVENT_FLAG_I2C_ABT) == TP_EVENT_FLAG_I2C_ABT )
        {
            *p_event = I2C_MASTER_EVENT_ABORTED;
        }
        else if( (statusbit & TP_EVENT_FLAG_I2C_RXC) == TP_EVENT_FLAG_I2C_RXC )
        {
            *p_event = I2C_MASTER_EVENT_RX_COMPLETE;
        }
        else if( (statusbit & TP_EVENT_FLAG_I2C_TXC) == TP_EVENT_FLAG_I2C_TXC )
        {
            *p_event = I2C_MASTER_EVENT_TX_COMPLETE;
        }
        else if( (statusbit & TP_EVENT_FLAG_I2C_ERR) == TP_EVENT_FLAG_I2C_ERR )
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
        printf("[EXC80] tx_event_flags_get failed(%X)\r\n", status);
        return TX_WAIT_ERROR;
    }
}


/**
 * @brief _exc80h60_i2c_scan
 */
static bool _exc80h60_i2c_scan( uint8_t *p_out )
{
    fsp_err_t err;
    UINT status;
    bool stat;
    i2c_master_event_t cb_event;

    exc80h60_i2c_start();

    err = p_i2c->p_api->read( p_i2c->p_ctrl, p_out, EXC80H_I2C_MAX_LEN, false );
    if( err != FSP_SUCCESS )
    {
        printf("read failed (%d)\r\n", err);
        stat = false;
    }
    else
    {
        status = _exc80h60_i2c_wait_cb( &cb_event );
        if( status != TX_SUCCESS )
        {
            printf("_exc80h60_i2c_wait_cb failed (%d)\r\n", status);
            stat = false;
        }
        else
        {
            switch(cb_event)
            {
            case I2C_MASTER_EVENT_RX_COMPLETE:
            case I2C_MASTER_EVENT_TX_COMPLETE:
                stat = true;
                break;
            case I2C_MASTER_EVENT_ABORTED:
            default:
                printf("[R]I2C_MASTER_EVENT_ABORTED or UNKNOWN (%d)\r\n", cb_event);
                stat = false;
            }
        }
    }

    exc80h60_i2c_stop();

    return stat;
}

static int8_t _exc80h60_analyze_report( uint8_t *p_in, tp_contact_info_t *p_contact )
{
    int8_t ret = 0;
    uint16_t rep_len = *((uint16_t*)p_in);      /* get length */
    uint8_t rep_id = *(p_in+2);                 /* get report id */
    uint8_t num_of_contacts = 0;
    uint32_t i;

    /* check report length */
    if( rep_len < EXC80H_I2C_MAX_LEN )
    {
        printf("report is too short(%d)\r\n", rep_len);
        return -1;
    }

    p_contact->report_id = rep_id;
    switch(rep_id)
    {
        case EXC80H_REP_ID_MT4K_WO_WH:          /* multi-touch 4k resolution with out W/H */
        case EXC80H_REP_ID_MT16K_WO_WH:         /* multi-touch 16k resolution with out W/H */
            num_of_contacts = *(p_in+3);
            if( num_of_contacts != 0 )
            {
                /* first contact info */
                p_contact->rem_of_contacts = (int8_t)num_of_contacts;
                p_contact->store_contacts = 0;
                if(num_of_contacts > 5)
                {
                    num_of_contacts = 5;
                }
            }
            else
            {
                if( p_contact->rem_of_contacts > 5 )
                {
                    num_of_contacts = 5;
                }
                else
                {
                    num_of_contacts = (uint8_t)p_contact->rem_of_contacts;
                }
            }

            /* save contact info */
            for( i=0; i<num_of_contacts; i++ )
            {
                memcpy( &(p_contact->report_info_10[p_contact->store_contacts]), (char*)(p_in+4+(i*sizeof(tp_10b_contact_t))), sizeof(tp_10b_contact_t) );
                p_contact->store_contacts++;
                p_contact->rem_of_contacts--;
            }
            if( _contact.rem_of_contacts <= 0 )
            {
                ret = p_contact->store_contacts;
            }
            break;
        case EXC80H_REP_ID_MT16K_W_WH:
            /* multi-touch 16k resolution with W/H */
            num_of_contacts = *(p_in+3);
            if( num_of_contacts != 0 )
            {
                /* first contact info */
                p_contact->rem_of_contacts = (int8_t)num_of_contacts;
                p_contact->store_contacts = 0;
                if(num_of_contacts > 4)
                {
                    num_of_contacts = 4;
                }
            }
            else
            {
                if( p_contact->rem_of_contacts > 4 )
                {
                    num_of_contacts = 4;
                }
                else
                {
                    num_of_contacts = (uint8_t)p_contact->rem_of_contacts;
                }
            }

            /* save contact info */
            for( i=0; i<num_of_contacts; i++ )
            {
                memcpy( &(p_contact->report_info_14[p_contact->store_contacts]), (char*)(p_in+4+(i*sizeof(tp_14b_contact_t))), sizeof(tp_14b_contact_t) );
                p_contact->store_contacts++;
                p_contact->rem_of_contacts--;
            }
            if( _contact.rem_of_contacts <= 0 )
            {
                ret = p_contact->store_contacts;
            }
            break;
        case EXC80H_REP_ID_AP4K:     /* active pen 4k resolution */
        case EXC80H_REP_ID_AP16K:    /* active pen 16k resolution */
            p_contact->reportinfo_apen.stat     = *(p_in+3);
            p_contact->reportinfo_apen.x_lsb    = *(p_in+4);
            p_contact->reportinfo_apen.x_msb    = *(p_in+5);
            p_contact->reportinfo_apen.y_lsb    = *(p_in+6);
            p_contact->reportinfo_apen.y_msb    = *(p_in+7);
            p_contact->reportinfo_apen.z_lsb    = *(p_in+8);
            p_contact->reportinfo_apen.z_msb    = *(p_in+9);
            p_contact->reportinfo_apen.x_tilt_lsb = *(p_in+10);
            p_contact->reportinfo_apen.x_tilt_msb = *(p_in+11);
            p_contact->reportinfo_apen.y_tilt_lsb = *(p_in+12);
            p_contact->reportinfo_apen.y_tilt_msb = *(p_in+13);
            ret = 1;
            break;
        default:
            printf("UNKNOWN report id(%d)\r\n", rep_id);
            ret = -1;
            memset( p_contact, 0x00, sizeof(tp_contact_info_t));
            break;
    }

    return ret;
}

static bool _exc80h60_analyze_touch( tp_contact_info_t *p_contact, tp_messege_info_t *p_out )
{
    uint16_t x,y;
    static tp_state_t old_state = TP_STATE_RELEASE;

    switch( p_contact->report_id )
    {
        case EXC80H_REP_ID_MT4K_WO_WH:
            if( p_contact->report_info_10[0].stat == 1 )
            {
                p_out->state = TP_STATE_PUSH;
            }
            else
            {
                p_out->state = TP_STATE_RELEASE;
            }
            x = (uint16_t)((p_contact->report_info_10[0].x_msb << 8) | p_contact->report_info_10[0].x_lsb);
            y = (uint16_t)((p_contact->report_info_10[0].y_msb << 8) | p_contact->report_info_10[0].y_lsb);
            p_out->pos_x = (uint16_t)((PANEL_MAX_X * x)/EXC80H_XY_RESO4K);
            p_out->pos_y = (uint16_t)((PANEL_MAX_Y * y)/EXC80H_XY_RESO4K);
            break;
        case EXC80H_REP_ID_MT16K_W_WH:
            if( p_contact->report_info_14[0].stat == 1 )
            {
                p_out->state = TP_STATE_PUSH;
            }
            else
            {
                p_out->state = TP_STATE_RELEASE;
            }
            x = (uint16_t)((p_contact->report_info_14[0].x_msb << 8) | p_contact->report_info_14[0].x_lsb);
            y = (uint16_t)((p_contact->report_info_14[0].y_msb << 8) | p_contact->report_info_14[0].y_lsb);
            p_out->pos_x = (uint16_t)((PANEL_MAX_X * x)/EXC80H_XY_RESO16K);
            p_out->pos_y = (uint16_t)((PANEL_MAX_Y * y)/EXC80H_XY_RESO16K);
            break;
        case EXC80H_REP_ID_MT16K_WO_WH:
            /* touch state update */
            switch(old_state)
            {
                case TP_STATE_PUSH:
                    if( p_contact->report_info_10[0].stat == 1 )
                    {
                        p_out->state = TP_STATE_DRAG;
                    }
                    else
                    {
                        p_out->state = TP_STATE_RELEASE;
                    }
                    old_state = p_out->state;
                    break;
                case TP_STATE_DRAG:
                    if( p_contact->report_info_10[0].stat == 1 )
                    {
                        p_out->state = TP_STATE_DRAG;
                    }
                    else
                    {
                        p_out->state = TP_STATE_RELEASE;
                    }
                    old_state = p_out->state;
                    break;
                case TP_STATE_RELEASE:
                default:
                    if( p_contact->report_info_10[0].stat == 1 )
                    {
                        p_out->state = TP_STATE_PUSH;
                    }
                    else
                    {
                        p_out->state = TP_STATE_RELEASE;
                    }
                    old_state = p_out->state;
                    break;
            }
#ifdef TP_USE_VERTICAL
            /* original contact info */
            x = (uint16_t)((p_contact->report_info_10[0].x_msb << 8) | p_contact->report_info_10[0].x_lsb);
            y = (uint16_t)((p_contact->report_info_10[0].y_msb << 8) | p_contact->report_info_10[0].y_lsb);
#else
            y = (uint16_t)((p_contact->report_info_10[0].x_msb << 8) | p_contact->report_info_10[0].x_lsb);
            x = (uint16_t)((p_contact->report_info_10[0].y_msb << 8) | p_contact->report_info_10[0].y_lsb);
            x = (uint16_t)abs(x - EXC80H_XY_RESO16K);
#endif
            p_out->pos_x = (uint16_t)((PANEL_MAX_X * x)/EXC80H_XY_RESO16K);
            p_out->pos_y = (uint16_t)((PANEL_MAX_Y * y)/EXC80H_XY_RESO16K);
            break;
        case EXC80H_REP_ID_AP4K:
            if(p_contact->reportinfo_apen.stat == 1 )
            {
                p_out->state = TP_STATE_PUSH;
            }
            else
            {
                p_out->state = TP_STATE_RELEASE;
            }
            x = (uint16_t)((p_contact->reportinfo_apen.x_msb << 8) | p_contact->reportinfo_apen.x_lsb);
            y = (uint16_t)((p_contact->reportinfo_apen.y_msb << 8) | p_contact->reportinfo_apen.y_lsb);
            p_out->pos_x = (uint16_t)((PANEL_MAX_X * x)/EXC80H_XY_RESO4K);
            p_out->pos_y = (uint16_t)((PANEL_MAX_Y * y)/EXC80H_XY_RESO4K);
            break;
        case EXC80H_REP_ID_AP16K:
            if(p_contact->reportinfo_apen.stat == 1 )
            {
                p_out->state = TP_STATE_PUSH;
            }
            else
            {
                p_out->state = TP_STATE_RELEASE;
            }
            x = (uint16_t)((p_contact->reportinfo_apen.x_msb << 8) | p_contact->reportinfo_apen.x_lsb);
            y = (uint16_t)((p_contact->reportinfo_apen.y_msb << 8) | p_contact->reportinfo_apen.y_lsb);
            p_out->pos_x = (uint16_t)((PANEL_MAX_X * x)/EXC80H_XY_RESO16K);
            p_out->pos_y = (uint16_t)((PANEL_MAX_Y * y)/EXC80H_XY_RESO16K);
            break;
        default:
            printf("[ERROR] unknown report id(%d)\r\n", p_contact->report_id);
            return false;
            break;
    }

    return true;
}

#endif /* TP_USE_EXC80H60 */
