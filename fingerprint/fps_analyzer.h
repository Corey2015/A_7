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
 * ��ȡһ��ͼ��, ֻ��һ��ԭʼͼȻ��ȥ��������������
 */
int get_one_image(uint8_t *imageData);


// image �ɼ�����ͼ������
//
// avg_frame ���ڴ����ԭʼͼ���� (��,�����ͼ��,ʹ�ö�����ԭʼͼ����ƽ��)
// num_frame �ܲɼ�������
// otsu_mul ����
// enh_range 0<= enh_range <= 255
// enh_img ���ͼ�������ͼ
int get_image(uint8_t avg_frame,
              uint8_t num_frame,
              double otsu_mul,
              uint32_t enh_range,
              uint8_t *enh_img);




// mode ģʽ:
//  mode == 0 : ʹ��Ĭ������ one,two,three����
//	mode == 1 : ����Ŀ�����޺����޵�У׼���ؼ�
//	    one --> upper bond
//	    two --> low bond
//	    three	--> ����
//
//	mode == 2 : ָ��Ҫɨ�����
//	    one--> first row
//	    two -->middle row
//	    three--> last row
//
//	mode == 3 : ָ��Ҫɨ��Ŀ�ʼ�ͽ���������
//	    one--> column index to begin scanning
//	    two--> column index to end scanning
//	    three --> ����
//
int calibrate_setting(uint8_t mode,uint8_t one,uint8_t two,uint8_t three);


int calibrate_image();




