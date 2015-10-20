//[*]--------------------------------------------------------------------------------------------------[*]
//
//
// 
//  I2C INA231(Sensor) driver
//  2013.07.17
// 
//
//[*]--------------------------------------------------------------------------------------------------[*]
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/platform_data/ina231.h>

//[*]--------------------------------------------------------------------------------------------------[*]
#include "ina231-misc.h"
#include "ina231-sysfs.h"
#ifdef CONFIG_CLOVERTRACE
#include "../staging/clovertrace/clovertrace.h"
#else
#define CLOVERTRACE_TIMESTAMP(...)
#endif

static spinlock_t spinlock;

//#define DEBUG_INA231
//[*]--------------------------------------------------------------------------------------------------[*]
//
// function prototype
//
//[*]--------------------------------------------------------------------------------------------------[*]
static 	void __exit		ina231_i2c_exit		(void);
static 	int __init 		ina231_i2c_init		(void);
static 	int  	        ina231_i2c_remove	(struct i2c_client *client);
static 	int  	        ina231_i2c_probe	(struct i2c_client *client, const struct i2c_device_id *id);
        int 	        ina231_i2c_read     (struct i2c_client *client, unsigned char cmd);
        int 	        ina231_i2c_write    (struct i2c_client *client, unsigned char cmd, unsigned short data);
        void            ina231_i2c_enable   (struct ina231_sensor *sensor);
static 	void 	        ina231_work		    (struct work_struct *work);

static enum hrtimer_restart ina231_timer    (struct hrtimer *timer);
		
//[*]--------------------------------------------------------------------------------------------------[*]
#ifdef CONFIG_PM
static int 	ina231_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	#ifdef CONFIG_HAS_EARLYSUSPEND
		struct	ina231	*sensor = i2c_get_clientdata(client);
	
		sensor->pdata->suspend(&client->dev);
	#endif

	return 0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int 	ina231_i2c_resume(struct i2c_client *client)
{
	#ifdef CONFIG_HAS_EARLYSUSPEND
		struct	ina231	*sensor = i2c_get_clientdata(client);

		sensor->pdata->resume(&cliet->dev);
	#endif	

	return 0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
#else
	#define ina231_i2c_suspend 	NULL
	#define ina231_i2c_resume  	NULL
#endif

//[*]--------------------------------------------------------------------------------------------------[*]
int 	ina231_i2c_read(struct i2c_client *client, unsigned char cmd)
{
	struct i2c_msg	msg[2];
	int 			ret;
	
	unsigned char   buf[2];

	memset(msg, 0x00, sizeof(msg));

	msg[0].addr 	= client->addr;
	msg[0].flags 	= 0;
	msg[0].len 		= 1;
	msg[0].buf 		= &cmd;

	msg[1].addr 	= client->addr;
	msg[1].flags    = I2C_M_RD;
	msg[1].len 		= 2;
	msg[1].buf 		= &buf[0];
	
	if ((ret = i2c_transfer(client->adapter, msg, 2)) != 2) {
		dev_err(&client->dev, "I2C read error: (%d) reg: 0x%X \n", ret, cmd);
		return -EIO;
	}

    ret = ((buf[0] << 8) | buf[1]) & 0xFFFF;
	return 	ret;
}

//[*]--------------------------------------------------------------------------------------------------[*]
int 	ina231_i2c_write(struct i2c_client *client, unsigned char cmd, unsigned short data)
{
	int 			ret;
	unsigned char	block_data[3];

	memset(block_data, 0x00, sizeof(block_data));

    block_data[0] = cmd;
    block_data[1] = (data >> 8) & 0xFF;	
    block_data[2] = (data     ) & 0xFF;	

	if ((ret = i2c_master_send(client, block_data, 3)) < 0) {
		dev_err(&client->dev, "I2C write error: (%d) reg: 0x%X \n", ret, cmd);
		return ret;
	}
	
	return ret;
}


// roger
int g_rthread_tid = -1;
int g_rthread_migration = 0;
int g_rthread_migration_b2l = 0;
int g_rthread_migration_l2b = 0;

static struct ina231_sensor *g_sensor[4];
static ktime_t g_start_time[4];

void ina231_i2c_enable_all(void)
{
	int i;
	g_start_time[0] = ktime_get();
	for (i = 0; i < 4; ++i) {
	    g_start_time[i] = g_start_time[0];
		g_sensor[i]->pd->enable = 1;
		g_sensor[i]->pd->pwrm.count = 0;	// roger
		g_sensor[i]->pd->pwrm.sum_power = 0;	// roger
//		g_sensor[i]->pd->pwrm.start_time = now;
		ina231_i2c_enable(g_sensor[i]);
	}
}
EXPORT_SYMBOL(ina231_i2c_enable_all);

void ina231_i2c_enable_sensor(int sensor_id)
{
    g_start_time[sensor_id] = g_start_time[0];
    g_sensor[sensor_id]->pd->enable = 1;
    g_sensor[sensor_id]->pd->pwrm.count = 0;    // roger
    g_sensor[sensor_id]->pd->pwrm.sum_power = 0;    // roger
    //      g_sensor[sensor_id]->pd->pwrm.start_time = now;
    ina231_i2c_enable(g_sensor[sensor_id]);
}
EXPORT_SYMBOL(ina231_i2c_enable_sensor);

void ina231_i2c_disable_all(void)
{
	int i;
	ktime_t now = ktime_get();
	s64 elapsed_time;

	elapsed_time = ktime_to_us(ktime_sub(now, g_start_time[0]));
	for (i = 0; i < 4; ++i) {
		g_sensor[i]->pd->enable = 0;
	    g_sensor[i]->pd->pwrm.elapsed_time = elapsed_time;
	}
}
EXPORT_SYMBOL(ina231_i2c_disable_all);

void ina231_i2c_disable_sensor(int sensor_id)
{
    ktime_t now = ktime_get();
    s64 elapsed_time;
    elapsed_time = ktime_to_us(ktime_sub(now, g_start_time[sensor_id]));
    g_sensor[sensor_id]->pd->enable = 0;
    g_sensor[sensor_id]->pd->pwrm.elapsed_time = elapsed_time;
}
EXPORT_SYMBOL(ina231_i2c_disable_sensor);

int64_t ina231_i2c_get_power_uW(int sensor_id)
{
    return  g_sensor[sensor_id]->pd->pwrm.sum_power / g_sensor[sensor_id]->pd->pwrm.count;
}
EXPORT_SYMBOL(ina231_i2c_get_power_uW);

//[*]--------------------------------------------------------------------------------------------------[*]
void    ina231_i2c_enable(struct ina231_sensor *sensor)
{
    hrtimer_start(&sensor->timer, ktime_set(sensor->timer_sec, sensor->timer_nsec), HRTIMER_MODE_REL);
}

//[*]--------------------------------------------------------------------------------------------------[*]
static 	void 	ina231_work		(struct work_struct *work)
{
	struct ina231_sensor 	*sensor = container_of(work, struct ina231_sensor, work);

    if(sensor->pd->enable)  {
        sensor->reg_bus_volt    = ina231_i2c_read(sensor->client, REG_BUS_VOLT   );
        sensor->reg_current     = ina231_i2c_read(sensor->client, REG_CURRENT    );
    
    	mutex_lock(&sensor->mutex);
        sensor->cur_uV = sensor->reg_bus_volt * FIX_uV_LSB;
        sensor->cur_uA = sensor->reg_current * sensor->cur_lsb_uA;
        sensor->cur_uW = (sensor->cur_uV / 1000 ) * (sensor->cur_uA / 1000);
        
        if((sensor->cur_uV > sensor->max_uV) || (sensor->cur_uA > sensor->cur_uA))  {
            sensor->max_uV = sensor->cur_uV;    sensor->max_uA = sensor->cur_uA;    sensor->max_uW = sensor->cur_uW;
        }
    	mutex_unlock(&sensor->mutex);

    	sensor->pd->pwrm.count++;	// roger
        sensor->pd->pwrm.sum_power += sensor->cur_uW;	// roger
#if 1
        {
        	// TODO: might have race condition
        	static int i = 0;
        	static int w = 0;

        	spin_lock(&spinlock);
        	w += sensor->cur_uW / 1000;
        	i++;
        	if (i == 4) {
        		CLOVERTRACE_TIMESTAMP(USER_GRAPH_ID, 5, w);
        		i = 0;
        		w = 0;
        	}
        	spin_unlock(&spinlock);
        }
#else
        {
        	int w;
        w += sensor->cur_uW / 1000;
    	spin_lock(&spinlock);
        if (strcmp(sensor->pd->name, "sensor_kfc") == 0) {
        		CLOVERTRACE_TIMESTAMP(USER_GRAPH_ID, 0, w);
        } else if (strcmp(sensor->pd->name, "sensor_arm") == 0) {
    		CLOVERTRACE_TIMESTAMP(USER_GRAPH_ID, 1, w);
        } else if (strcmp(sensor->pd->name, "sensor_g3d") == 0) {
    		CLOVERTRACE_TIMESTAMP(USER_GRAPH_ID, 2, w);
        } else if (strcmp(sensor->pd->name, "sensor_mem") == 0) {
    		CLOVERTRACE_TIMESTAMP(USER_GRAPH_ID, 3, w);
        }
    	spin_unlock(&spinlock);
        }
#endif

    }
    else    {
        sensor->cur_uV = 0; sensor->cur_uA = 0; sensor->cur_uW = 0;
    }
    

//    printk("DS: pwr=%4d mW, 0x%04X 0x%04X\n",
//    		sensor->cur_uW / 1000,
//			ina231_i2c_read(sensor->client, REG_ALERT_EN),
//			ina231_i2c_read(sensor->client, REG_CONFIG));
//    printk("DS: pwr=%4d uW\n", sensor->cur_uW);
#if defined(DEBUG_INA231)
    printk("%s : BUS Voltage = %06d uV, %1d.%06d V\n", sensor->pd->name, sensor->cur_uV, sensor->cur_uV/1000000, sensor->cur_uV%1000000);
    printk("%s : Curent      = %06d uA, %1d.%06d A\n", sensor->pd->name, sensor->cur_uA, sensor->cur_uA/1000000, sensor->cur_uA%1000000);
    printk("%s : Powert      = %06d uW, %1d.%06d W\n", sensor->pd->name, sensor->cur_uW, sensor->cur_uW/1000000, sensor->cur_uW%1000000);
#endif    
}

//[*]--------------------------------------------------------------------------------------------------[*]
static enum hrtimer_restart ina231_timer(struct hrtimer *timer)
{
	struct ina231_sensor 	*sensor = container_of(timer, struct ina231_sensor, timer);

    queue_work(sensor->wq, &sensor->work);
	
    if(sensor->pd->enable)  ina231_i2c_enable(sensor);
	
	return HRTIMER_NORESTART;
}

//[*]--------------------------------------------------------------------------------------------------[*]
#ifdef CONFIG_OF
static int  ina231_i2c_dt_parse(struct i2c_client *client, struct ina231_sensor *sensor)
{
    struct device_node *sensor_np = client->dev.of_node;
    const char  *sensor_name;
    unsigned int    rdata;

	if(!(sensor->pd = devm_kzalloc(&client->dev, sizeof(struct ina231_pd), GFP_KERNEL)))	{
		dev_err(&client->dev, "INA231 Sensor platform data struct malloc error!\n");
		return	-ENOMEM;
	}

	if (of_property_read_string(sensor_np, "sensor-name", &sensor_name))    return  -1;
    sensor->pd->name = (unsigned char *)sensor_name;
	
	if (of_property_read_u32(sensor_np, "enable", &rdata))                  return  -1;
	sensor->pd->enable = rdata;
	
	if (of_property_read_u32(sensor_np, "max_A", &rdata))                   return  -1;
	sensor->pd->max_A = rdata;
	
	if (of_property_read_u32(sensor_np, "shunt_R_mohm", &rdata))            return  -1;
	sensor->pd->shunt_R_mohm = rdata;
	
	if (of_property_read_u32(sensor_np, "config", &rdata))                  return  -1;
	sensor->pd->config = rdata;

	/* 0x45FB: 0100 0101 1111 1011
	 *  RST: 0100: none
	 *  AVG:  010: 16 times
	 * Vbus:  111: 8.244 ms
	 *  Vsh:  111: 8.244 ms
	 * Mode:  011: shunt, bus, triggered
	*/
	sensor->pd->config = 0x45FF;	// roger
//	sensor->pd->config = 0x45FB;	// roger
	
	if (of_property_read_u32(sensor_np, "update_period", &rdata))           return  -1;
//	sensor->pd->update_period = rdata;
	sensor->pd->update_period = 125000;//131904; // roger: overhead exists so set to higher frequency
//	sensor->pd->update_period = 10000;//131904; // roger: overhead exists so set to higher frequency

    return  0;
}

#else   // CONFIG_OF
static int  ina231_i2c_dt_parse(struct i2c_client *client, struct ina231_sensor *sensor)
{
    return  0;
}
#endif

//[*]--------------------------------------------------------------------------------------------------[*]
static int 	ina231_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int     rc = 0;
    struct  ina231_sensor   *sensor;

	if(!(sensor = devm_kzalloc(&client->dev, sizeof(struct ina231_sensor), GFP_KERNEL)))	{
		dev_err(&client->dev, "INA231 Sensor struct malloc error!\n");
		return	-ENOMEM;
	}

    // mutex init	
	mutex_init(&sensor->mutex);

	sensor->client	= client;

    if (client->dev.of_node)    {
        if(ina231_i2c_dt_parse(client, sensor) < 0)    goto     out;
    }
    else    {
        sensor->pd = client->dev.platform_data;
    }
	
	i2c_set_clientdata(client, sensor);

    // Calculate current lsb value
    sensor->cur_lsb_uA  = sensor->pd->max_A * 1000000 / 32768;
    // Calculate register value
    sensor->reg_calibration = 5120000 / (sensor->cur_lsb_uA * sensor->pd->shunt_R_mohm);

    if((rc = ina231_i2c_write(sensor->client, REG_CONFIG,      sensor->pd->config))        < 0) goto out;
    if((rc = ina231_i2c_write(sensor->client, REG_CALIBRATION, sensor->reg_calibration))   < 0) goto out;
    if((rc = ina231_i2c_write(sensor->client, REG_ALERT_EN,    0x0000))  < 0)                   goto out;
//    if((rc = ina231_i2c_write(sensor->client, REG_ALERT_EN,    0x0400))  < 0)                   goto out;	// roger
    if((rc = ina231_i2c_write(sensor->client, REG_ALERT_LIMIT, 0x0000))  < 0)                   goto out;
    
    if((rc = ina231_i2c_read(sensor->client, REG_CONFIG      )) != sensor->pd->config      )    goto out;
    if((rc = ina231_i2c_read(sensor->client, REG_CALIBRATION )) != sensor->reg_calibration )    goto out;
    if((rc = ina231_i2c_read(sensor->client, REG_ALERT_EN    )) != 0x0000)                      goto out;
    if((rc = ina231_i2c_read(sensor->client, REG_ALERT_LIMIT )) != 0x0000)                      goto out;

    // misc driver probe
    if(ina231_misc_probe(sensor) < 0)           goto out;

    // sysfs probe
    if(ina231_sysfs_create(&client->dev) < 0)   goto out;

    // timer run for sensor data receive
    INIT_WORK(&sensor->work, ina231_work);
    if((sensor->wq = create_singlethread_workqueue("ina231_wq")) == NULL)	goto out;
        
    hrtimer_init(&sensor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    sensor->timer_sec  = sensor->pd->update_period / 1000000;
    sensor->timer_nsec = sensor->pd->update_period % 1000000;
    sensor->timer_nsec = sensor->timer_nsec * 1000;
    sensor->timer.function = ina231_timer;

    if(sensor->pd->enable)  ina231_i2c_enable(sensor);

    // display register message
    rc = 0;
    dev_info(&client->dev, "============= Probe INA231 : %s ============= \n", sensor->pd->name);
    dev_info(&client->dev, "SENSOR ENABLE   : %s\n"     , sensor->pd->enable ? "true" : "false");
    dev_info(&client->dev, "REG CONFIG      : 0x%04X\n" , sensor->pd->config        );
    dev_info(&client->dev, "REG CALIBRATION : 0x%04X\n" , sensor->reg_calibration   );
    dev_info(&client->dev, "SHUNT Resister  : %d mOhm\n", sensor->pd->shunt_R_mohm  );
    dev_info(&client->dev, "MAX Current     : %d A\n"   , sensor->pd->max_A         );
    dev_info(&client->dev, "Current LSB uA  : %d uA\n"  , sensor->cur_lsb_uA        );
    dev_info(&client->dev, "Conversion Time : %d us\n"  , sensor->pd->update_period );
    dev_info(&client->dev, "=====================================================\n");
    
    // roger
    if (0 == strcmp(sensor->pd->name + 7, "arm")) {
    	g_sensor[0] = sensor;
    } else if (0 == strcmp(sensor->pd->name + 7, "kfc")) {
    	g_sensor[1] = sensor;
    } else if (0 == strcmp(sensor->pd->name + 7, "g3d")) {
    	g_sensor[2] = sensor;
    } else if (0 == strcmp(sensor->pd->name + 7, "mem")) {
    	g_sensor[3] = sensor;
    }
    return  0;
out:
    dev_err(&client->dev, "============= Probe INA231 Fail! : %s (0x%04X) ============= \n", sensor->pd->name, rc); 

	return rc;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static int 	ina231_i2c_remove(struct i2c_client *client)
{
    struct  ina231_sensor   *sensor = dev_get_drvdata(&client->dev);

    // removed sysfs entry
    ina231_sysfs_remove (&client->dev);
    // removed misc drv
	ina231_misc_remove	(&client->dev);
    // timer
    if(sensor->pd->enable)  hrtimer_cancel(&sensor->timer);

    return  0;
}

//[*]--------------------------------------------------------------------------------------------------[*]
static const struct i2c_device_id ina231_id[] = {
	{ INA231_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ina231_id);

//[*]--------------------------------------------------------------------------------------------------[*]
static struct i2c_driver ina231_i2c_driver = {
	.driver = {
		.name	= INA231_I2C_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ina231_i2c_probe,
	.remove		= ina231_i2c_remove,
	.suspend	= ina231_i2c_suspend,
	.resume		= ina231_i2c_resume,
	.id_table	= ina231_id,
};

//[*]--------------------------------------------------------------------------------------------------[*]
static int __init 	ina231_i2c_init(void)
{
	printk(KERN_INFO"ina231_i2c modified by Roger is being loaded!\n");
    spin_lock_init(&spinlock);

	return i2c_add_driver(&ina231_i2c_driver);
}
module_init(ina231_i2c_init);

//[*]--------------------------------------------------------------------------------------------------[*]
static void __exit 	ina231_i2c_exit(void)
{

	i2c_del_driver(&ina231_i2c_driver);
}
module_exit(ina231_i2c_exit);

//[*]--------------------------------------------------------------------------------------------------[*]
//[*]--------------------------------------------------------------------------------------------------[*]
