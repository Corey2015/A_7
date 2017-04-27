#include <stdint.h>
#include "fpslinuxdriver.h"
#include "linuxfps.h"
#include "et300_reg.h"



////////////////////////////////////////////////////////////////////////////////
//
// Menu Functions
//

int open_sensor();
int close_sensor();
int init_sensor(int);
int read_register(uint8_t addr,uint8_t *data);
int write_register(uint8_t addr,uint8_t data);

int get_original_image(uint8_t *image);
int get_save_background();


// Get current getting image settings
int get_current_settings();

/*
 * 获取一张图像, 只采一张原始图然后去除背景，最后输出
 */
int get_one_image(uint8_t *imageData);


// image 采集到的图像数据
//
// avg_frame 用于处理的原始图像数 (即,输出的图像,使用多少张原始图进行平均)
// num_frame 总采集的组数
// otsu_mul 增益
// enh_range 0<= enh_range <= 255
// enh_img 输出图像的增加图
int get_image(uint8_t avg_frame,
              uint8_t num_frame,
              double otsu_mul,
              uint32_t enh_range,
              uint8_t *enh_img);




// mode 模式:
//  mode == 0 : 使用默认设置 one,two,three备用
//	mode == 1 : 设置目标上限和下限的校准像素键
//	    one --> upper bond
//	    two --> low bond
//	    three	--> 备用
//
//	mode == 2 : 指定要扫描的行
//	    one--> first row
//	    two -->middle row
//	    three--> last row
//
//	mode == 3 : 指定要扫描的开始和结束列索引
//	    one--> column index to begin scanning
//	    two--> column index to end scanning
//	    three --> 备用
//
int calibrate_setting(uint8_t mode,uint8_t one,uint8_t two,uint8_t three);


int calibrate_image();




