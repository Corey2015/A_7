//
// Created by linrb on 7/6/16.
//

#ifndef FP_CORE_V4_FP_API_H
#define FP_CORE_V4_FP_API_H

#ifdef __cplusplus
extern "C" {
#endif
#include "ufvp_common.h"

#define ALG_RTN_OK                     0
#define ALG_RTN_INPUTNULL             (-1 )  	//输入指针为空
#define ALG_RTN_HWOVERFLOW            (-2 )  	//输入图像大小溢出
#define ALG_RTN_FEATUREOVERFLOW       (-3 )  	//mp个数越界
#define ALG_RTN_SAMPLEOVERFLOW        (-4 )  	//模板个数越界
#define ALG_RTN_DIMHOVERFLOW          (-5 )  	//transM个数越界
#define ALG_RTN_CRCFAILED             (-6 )  	//crc校验错误
#define ALG_RTN_IMGERROR              (-7 )  	//输入图像宽高溢出

#define ALG_RTN_LENGHTERROR			  (-8)		// 输入长度出错
#define ALG_RTN_NAMEERROR			  (-9)		// 输入传感器类型名字长度溢出



typedef struct
{
    char* sensor_type;     //传感器类型
	int   finger_area;     //手指覆盖面积百分比，范围0-100
	int   finger_status;   //手指干湿状态，范围0-100，值越小代表手指越湿,返回-1则功能未打开
} PEXTERN_PARA;  //pExternPar for FeatureExtract

typedef struct
{
    int   x;
    int   y;
} POINT_ARRAY;

typedef struct
{
    int   num_single;       //num_single:独立模板个数
    int   earlier_success;  //earlier_success:是否提前结束：1代表成功，0代表不提前成功
    int   overlap_ratio;    //overlap_ratio覆盖率：0-100（从第二次开始）手指重合度过高提示，推荐阈值为>=40
    int   points_en;         //下面输出的坐标时能信号，1为使能
    POINT_ARRAY img_point[4]; //图像四个顶点坐标，存储顺序为：左下,右下,右上,左上
} PEXTERN_REG_PARA;  //pExternPar for FeatureEnroll

typedef struct
{
	int   update_en;       //模板更新使能，小于0时不更新
	int   points_en;         //下面输出的坐标时能信号，1为使能
	POINT_ARRAY img_point[4]; //图像四个顶点坐标，存储顺序为：左下,右下,右上,左上
} PEXTERN_MATCH_PARA;  //pExternPar for FeatureMatch

	typedef struct {

		ufvp_module_t	common;

		/**
		 * @brief	对指纹图像数据进行特征提取,生成指纹特征数据
		 *
         * @Param     pFingerImgBuf		指纹图像数据指针,输入参数
         * @Param     nWidth	        图像的宽度信息,输入参数
         * @Param     nHeight	    	图像的高度信息,输入参数
         * @Param     pFeatureData		指纹特征数据指针,存储生成的指纹特征数据,由调用者分配内存空间,输出参数，固定大小：7*1024BYTE+一幅图像大小
         * @Param     pExternPar	    输出“PEXTERN_PARA”类型结构体
         *                              sensor_type：传感器类型，由外部提供
	                                    finger_area：手指覆盖面积百分比，范围0-100，由算法输出
	                                    finger_status：手指干湿状态，范围0-100，由算法输出
         *
         *
         * @retval    成功:返回0；<0 失败:返回错误代码
		*/

		int(*FeatureExtract)(
			unsigned char	*pFingerImgBuf,
			unsigned int	nWidth,
			unsigned int	nHeight,
			unsigned char	*pFeatureData,
			void			*pExternPar);

         /**
          * @brief     将指纹特征数据与数据库中的指纹特征数据比对,得到相似度
          *
          * @Param     pFeatureData	     指纹特征数据指针,输入参数，固定大小：7*1024BYTE+一幅图像大小
          * @Param     pFeatureDataBase  数据库中指纹特征数据指针,输入参数，固定大小：512*1024BYTE
          * @Param     pfSimilarity      相似度,取值范围为0.00~1.00,值0.00表示不匹配,值1.00表示完全匹配,输出参数  >=0.25表示比对成功
          * @Param     pExternPar	     输出"PEXTERN_MATCH_PARA"类型结构体
                                         int update_en:模板更新状态,小于零则不更新 ,大于等于零更新
                                         points_en：下面输出的坐标时能信号，大于0为使能
                                         img_point[4]：图像四个顶点坐标，存储顺序为：左下,右下,右上,左上
          *
          * @retval      成功:返回0;  <0 失败:返回错误代码
 							-1	-- 输入指针为空
							-2	-- 输入图像大小溢出
							-7  -- 输入图像宽高溢出
         */

		int(*FeatureMatch)(
			unsigned char	*pFeatureData,
			unsigned char	*pFeatureDataBase,
			float			*pfSimilarity,
			void			*pExternPar);

		/**
         * @brief     合并指纹特征数据到数据库中的指纹特征数据
         *
         * @Param     pFeatureData	     指纹特征数据指针,输入参数，固定大小：7*1024BYTE+一幅图像大小
         * @Param     pFeatureDataBase   数据库中指纹特征数据指针,输入参数,输入参数，固定大小：512*1024BYTE
         * @Param     pExternPar	     输出"PEXTERN_REG_PARA"类型结构体
                                         num_single:独立模板个数
                                         earlier_success:是否提前结束：1代表成功，0代表不提前成功
                                         overlap_ratio覆盖率：0-100（从第四次开始）手指重合度过高提示，推荐阈值为40
                                         points_en：下面输出的坐标时能信号，1为使能
                                         img_point[4]：图像四个顶点坐标，存储顺序为：左下,右下,右上,左上
         *
         * @retval      成功:返回0；<0 失败:返回错误代码
		 */
		int(*FeatureEnroll)(
			unsigned char	*pFeatureData,
			unsigned char	*pFeatureDataBase,
			void			*pExternPar);


	}fp_core_t;

extern fp_core_t fp_core;

#ifdef __cplusplus
}
#endif

#endif //FP_CORE_V4_FP_API_H
