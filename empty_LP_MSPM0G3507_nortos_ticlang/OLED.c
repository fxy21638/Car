#include "OLED.h"
#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "string.h"
#include <stdarg.h>

#define Soft_OLED_I2C // 使用软件I2C
// #define Hard_OLED_I2C   // 使用硬件I2C
// #define Soft_OLED_SPI   // 使用软件SPI
// #define Hard_OLED_SPI   // 使用硬件SPI

uint8_t OLED_DisplayBuf[8][128];

/*
 * 软件I2C时序延迟：
 * 当前软件模拟I2C没有加入时钟，GPIO翻转很快，OLED容易通信失败。
 * 这里通过在SCL边沿加入稳定延时，值越大越慢。
 */
#ifndef OLED_I2C_DELAY_CYCLES
#define OLED_I2C_DELAY_CYCLES (200U)
#endif

static inline void OLED_I2C_Delay(void)
{
	delay_cycles(OLED_I2C_DELAY_CYCLES);
}

/********************* 全局变量 */

/* 函数定义 *********************/

#ifdef Soft_OLED_I2C

/*
 * 注意：
 * 当前默认使用软件模拟I2C驱动OLED。
 * 如需要修改为PA0/PA1：
 *   - SDA = PA0
 *   - SCL = PA1
 * 请使用SysConfig生成的 OLED_Pin_* 宏定义，以保持与硬件接线一致的配置。
 */
/* 使用 SysConfig 生成的 Pin Group 定义，避免和 .syscfg 不一致 */
#define OLED_I2C_PORT (OLED_Pin_PORT)
#define OLED_I2C_SDA_PIN (OLED_Pin_SDA_PIN)
#define OLED_I2C_SCL_PIN (OLED_Pin_SCL_PIN)
#define OLED_I2C_SDA_IOMUX (OLED_Pin_SDA_IOMUX)
#define OLED_I2C_SCL_IOMUX (OLED_Pin_SCL_IOMUX)

static inline void OLED_SCL_Write(uint8_t level)
{
	if (level)
	{
		DL_GPIO_setPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
	}
	else
	{
		DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
	}
}

/*
 * SDA 开漏模拟：
 * - 写 0：配置为输出并拉低
 * - 写 1：配置为输入(释放总线)，依靠内部上拉把线拉高
 */
static inline void OLED_SDA_DriveLow(void)
{
	DL_GPIO_clearPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
	DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
}

static inline void OLED_SDA_Release(void)
{
	/* 关闭输出使其变为输入(高阻)，由上拉电阻拉高 */
	DL_GPIO_disableOutput(OLED_I2C_PORT, OLED_I2C_SDA_PIN);
}

static inline void OLED_SDA_Write(uint8_t level)
{
	if (level)
	{
		OLED_SDA_Release();
	}
	else
	{
		OLED_SDA_DriveLow();
	}
}

static inline uint8_t OLED_SDA_Read(void)
{
	return (DL_GPIO_readPins(OLED_I2C_PORT, OLED_I2C_SDA_PIN) != 0U) ? 1U : 0U;
}

/* 兼容旧宏名 */
#define OLED_W_SCL(x) (OLED_SCL_Write((x) ? 1U : 0U))
#define OLED_W_SDA(x) (OLED_SDA_Write((x) ? 1U : 0U))

#endif

#ifdef Soft_OLED_SPI

#define OLED_W_D0(x) ((x) ? (DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_16)) : (DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_16)))
#define OLED_W_D1(x) ((x) ? (DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_15)) : (DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_15)))
#define OLED_W_RES(x) ((x) ? (DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_17)) : (DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17)))
#define OLED_W_DC(x) ((x) ? (DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_1)) : (DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_1)))
#define OLED_W_CS(x) ((x) ? (DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_29)) : (DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_29)))

#endif

#ifdef Hard_OLED_SPI

#define OLED_RES(x) ((x) ? (DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_17)) : (DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_17)))

#endif

/**
 * 函   数：OLED引脚初始化
 * 参   数：无
 * 返 回 值：无
 * 说   明：此上层函数在需要初始化时会被调用
 *           用户需要将SCL和SDA引脚初始化为开漏模式并释放总线
 */
void OLED_GPIO_Init(void)
{

#ifdef Soft_OLED_I2C
	//    /* 将SCL和SDA引脚初始化为开漏模式 */
	//   RCC->APB2ENR |= RCC_APB2Periph_GPIOB;/* GPIOB时钟使能 */
	//   sys_gpio_set(GPIOB, SYS_GPIO_PIN6 | SYS_GPIO_PIN7,
	//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_OD, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);   /* LED0输出模式配置 */
	/*
	 * 内部上拉：没有外接上拉电阻时，只能依赖 MCU 内部上拉(较弱，建议降速)。
	 * - SDA 设为输入+上拉，用于“释放为高”与 ACK 读取
	 * - SCL 仍用推挽输出(拉高/拉低)
	 */
	DL_GPIO_initDigitalInputFeatures(OLED_I2C_SDA_IOMUX,
									 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
									 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
	DL_GPIO_initDigitalOutput(OLED_I2C_SCL_IOMUX);

	DL_GPIO_enableOutput(OLED_I2C_PORT, OLED_I2C_SCL_PIN);
	OLED_SDA_Release();

	/* 空闲态：SCL=1, SDA=1(释放) */
	OLED_W_SCL(1);
#endif

#ifdef Soft_OLED_SPI
	//   RCC->APB2ENR |= 1 << 2;/* GPIOA时钟使能 */
	//   sys_gpio_set(GPIOA, SYS_GPIO_PIN12 | SYS_GPIO_PIN15,
	//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);

	//
	//   RCC->APB2ENR |= 1 << 3;/* GPIOB时钟使能 */
	//   sys_gpio_set(GPIOB, SYS_GPIO_PIN3 | SYS_GPIO_PIN4 |SYS_GPIO_PIN5,
	//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);

	/* 配置默认电平 */
	OLED_W_D0(0);
	OLED_W_D1(1);
	OLED_W_RES(1);
	OLED_W_DC(1);
	OLED_W_CS(1);
#endif

#ifdef Hard_OLED_SPI
	delay_ms(100);
	OLED_RES(0);
	delay_ms(20);
	OLED_RES(1);

#endif
}

/********************* 函数定义 */

/* 通信协议 *********************/
#ifdef Soft_OLED_I2C

/**
 * 函   数：I2C起始
 * 参   数：无
 * 返 回 值：无
 */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1); // 释放SDA
	OLED_W_SCL(1); // SCL拉高
	OLED_I2C_Delay();
	OLED_W_SDA(0); // SCL为高时，SDA拉低产生START
	OLED_I2C_Delay();
	OLED_W_SCL(0); // 占用总线
	OLED_I2C_Delay();
}

/**
 * 函   数：I2C终止
 * 参   数：无
 * 返 回 值：无
 */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0); // SDA拉低
	OLED_I2C_Delay();
	OLED_W_SCL(1); // SCL拉高
	OLED_I2C_Delay();
	OLED_W_SDA(1); // SCL为高时释放SDA产生STOP
	OLED_I2C_Delay();
}
#endif
/**
 * 函   数：I2C发送一个字节
 * 参   数：Byte 要发送的一个字节数据，范围：0x00~0xFF
 * 返 回 值：无
 */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
#ifdef Soft_OLED_I2C
	/* 循环8次，依次发送数据的每一位 */
	for (i = 0; i < 8; i++)
	{
		/* 使用运算的方式取出Byte的指定位数据并写入到SDA线 */
		/* 两个!!运算符，将非零值转为1 */
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_I2C_Delay();
		OLED_W_SCL(1); // 释放SCL，从机在SCL高电平期间读取SDA
		OLED_I2C_Delay();
		OLED_W_SCL(0); // 拉低SCL，准备开始发送下一位数据
		OLED_I2C_Delay();
	}

	/* 第9位：ACK，释放SDA让从机拉低 */
	OLED_W_SDA(1);
	OLED_I2C_Delay();
	OLED_W_SCL(1);
	OLED_I2C_Delay();
	(void)OLED_SDA_Read();
	OLED_W_SCL(0);
	OLED_I2C_Delay();
#endif
#ifdef Soft_OLED_SPI
	/* 循环8次，依次发送数据的每一位 */
	for (i = 0; i < 8; i++)
	{
		/* 使用运算的方式取出Byte的指定位数据并写入到D1线 */
		/* 两个!!运算符，将非零值转为1 */
		OLED_W_D1(!!(Byte & (0x80 >> i)));
		OLED_W_D0(1); // 拉高D0，从机在D0高电平期间读取SDA
		OLED_W_D0(0); // 拉低D0，准备开始发送下一位数据
	}
#endif
#ifdef Hard_OLED_SPI
	SPI_WriteByte(Byte);
#endif
}

/**
 * 函   数：OLED写命令
 * 参   数：Command 要写入的命令值，范围：0x00~0xFF
 * 返 回 值：无
 */
void OLED_WriteCommand(uint8_t Command)
{
#ifdef Soft_OLED_I2C
	OLED_I2C_Start();			// I2C起始
	OLED_I2C_SendByte(0x78);	// 发送OLED的I2C从机地址
	OLED_I2C_SendByte(0x00);	// 控制字节，写0x00表示后面写命令
	OLED_I2C_SendByte(Command); // 写入指定命令
	OLED_I2C_Stop();			// I2C终止
#endif
#ifdef Hard_OLED_I2C
	i2c0_write_n_byte(0x3C, 0x00, &Command, 1);
#endif
#ifdef Soft_OLED_SPI
	OLED_W_CS(0);				// 拉低CS，开始通信
	OLED_W_DC(0);				// 拉低DC，表示后面是命令
	OLED_I2C_SendByte(Command); // 写入指定命令
	OLED_W_CS(1);				// 拉高CS，结束通信
#endif

#ifdef Hard_OLED_SPI
	SPI_Send_cmd(Command);
#endif
}

/**
 * 函   数：OLED写数据
 * 参   数：Data 要写入数据的起始地址
 * 参   数：Count 要写入数据的个数
 * 返 回 值：无
 */
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
	uint8_t i;
#ifdef Soft_OLED_I2C
	OLED_I2C_Start();		 // I2C起始
	OLED_I2C_SendByte(0x78); // 发送OLED的I2C从机地址
	OLED_I2C_SendByte(0x40); // 控制字节，写0x40表示后面写数据
	/* 循环Count次，连续发送数据写入 */
	for (i = 0; i < Count; i++)
	{
		OLED_I2C_SendByte(Data[i]); // 依次发送Data的每一个数据
	}
	OLED_I2C_Stop(); // I2C终止
#endif
#ifdef Hard_OLED_I2C
	i2c0_write_n_byte(0x3C, 0x40, Data, Count);
#endif
#ifdef Soft_OLED_SPI
	OLED_W_CS(0); // 拉低CS，开始通信
	OLED_W_DC(1); // 拉高DC，表示后面是数据
	/* 循环Count次，连续发送数据写入 */
	for (i = 0; i < Count; i++)
	{
		OLED_I2C_SendByte(Data[i]); // 依次发送Data的每一个数据
	}
	OLED_W_CS(1); // 拉高CS，结束通信
#endif

#ifdef Hard_OLED_SPI
	for (i = 0; i < Count; i++)
	{
		SPI_WriteByte(Data[i]); // 依次发送Data的每一个数据
	}
#endif
}

/********************* 通信协议 */

/* 硬件配置 *********************/

/**
 * 函   数：OLED初始化
 * 参   数：无
 * 返 回 值：无
 * 说   明：使用前必须调用此初始化函数
 */
void OLED_Init(void)
{
	OLED_GPIO_Init(); // 先调用底层的端口初始化

	/* 写入一系列的命令，对OLED进行初始化配置 */
	OLED_WriteCommand(0xAE); // 设置显示开/关：0xAE关闭，0xAF打开

	OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80); // 0x00~0xFF

	OLED_WriteCommand(0xA8); // 设置多路复用比
	OLED_WriteCommand(0x3F); // 0x0E~0x3F

	OLED_WriteCommand(0xD3); // 设置显示偏移
	OLED_WriteCommand(0x00); // 0x00~0x7F

	OLED_WriteCommand(0x40); // 设置显示起始行：0x40~0x7F

	/* Memory addressing mode: Page addressing (match OLED_SetCursor + 0xB0 page commands) */
	OLED_WriteCommand(0x20);
	OLED_WriteCommand(0x02);

	OLED_WriteCommand(0xA1); // 设置左右方向：0xA1正常，0xA0左右反置

	OLED_WriteCommand(0xC8); // 设置上下方向：0xC8正常，0xC0上下反置

	OLED_WriteCommand(0xDA); // 设置COM引脚硬件配置
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81); // 设置对比度
	OLED_WriteCommand(0xCF); // 0x00~0xFF

	OLED_WriteCommand(0xD9); // 设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB); // 设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4); // 设置全局显示开/关

	OLED_WriteCommand(0xA6); // 设置正常/反色显示：0xA6正常，0xA7反色

	OLED_WriteCommand(0x8D); // 设置电荷泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF); // 开启显示

	/* 上电后强制清屏并刷一次，避免 OLED 内部 GDDRAM 上电态导致整屏常亮 */
	OLED_Clear();
	OLED_Update();
}

/**
 * 函   数：OLED设置显示光标位置
 * 参   数：Page 指定光标所在的页，范围：0~7
 * 参   数：X 指定光标所在的X轴坐标，范围：0~127
 * 返 回 值：无
 * 说   明：对于OLED默认的Y轴，只有8个Bit为一组写入，即1页包含8个Y轴像素
 */
void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	/* 如果使用此程序驱动1.3寸的OLED显示屏，需要注意以下注释 */
	/* 因为1.3寸的OLED驱动芯片是SH1106，有132列 */
	/* 屏幕的起始列偏移到了第2列，而不是第0列 */
	/* 所以需要将X加2，才能正常显示 */
	//    X += 2;

	/* 通过指令设置页地址和列地址 */
	OLED_WriteCommand(0xB0 | Page);				 // 设置页位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); // 设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));		 // 设置X位置低4位
}

/********************* 硬件配置 */

/* 工具函数 *********************/

/* 工具函数，仅内部部分函数使用 */

/**
 * 函   数：OLED幂函数
 * 参   数：X 底数
 * 参   数：Y 指数
 * 返 回 值：等于X的Y次方
 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1; // 结果默认为1
	while (Y--)			 // 累乘Y次
	{
		Result *= X; // 每次把X累乘到结果
	}
	return Result;
}

/**
 * 函   数：判断指定点是否在指定多边形内部
 * 参   数：nvert 多边形的顶点数
 * 参   数：vertx verty 多边形顶点的x和y坐标数组
 * 参   数：testx testy 测试点的x和y坐标
 * 返 回 值：指定点是否在指定多边形内部：1在内部，0不在内部
 */
uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i, j, c = 0;

	/* 此算法由 W. Randolph Franklin 提出 */
	/* 参考链接：https://wrfranklin.org/Research/Short_Notes/pnpoly.html */
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
		{
			c = !c;
		}
	}
	return c;
}

/**
 * 函   数：判断指定点是否在指定角度内部
 * 参   数：X Y 指定点坐标
 * 参   数：StartAngle EndAngle 起始角度和终止角度，范围：-180~180
 *           水平向右为0度，水平向左为180度或-180度，下方为正，上方为负，顺时针旋转
 * 返 回 值：指定点是否在指定角度内部：1在内部，0不在内部
 */
uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle)
{
	int16_t PointAngle;
	PointAngle = atan2(Y, X) / 3.14 * 180; // 计算指定点的弧度，并转换为角度表示
	if (StartAngle < EndAngle)			   // 起始角度小于终止角度的情况
	{
		/* 如果指定角度在起始终止角度之间，则判断指定点在指定角度 */
		if (PointAngle >= StartAngle && PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	else // 起始角度大于终止角度的情况
	{
		/* 如果指定角度大于起始角度或小于终止角度，则判断指定点在指定角度 */
		if (PointAngle >= StartAngle || PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	return 0; // 其余情况，根据以上判断指定点不在指定角度
}

/********************* 工具函数 */

/* 显示函数 *********************/

/**
 * 函   数：将OLED显存数组更新到OLED屏幕
 * 参   数：无
 * 返 回 值：无
 * 说   明：所有的显示函数都只是对OLED显存数组进行读写
 *           需要调用OLED_Update或OLED_UpdateArea函数
 *           才会将显存数组的数据发送到OLED硬件并触发显示
 *           故每次显示操作之后想同步到屏幕上，需要调用更新函数
 */
void OLED_Update(void)
{
	uint8_t j;
	/* 遍历每一页 */
	for (j = 0; j < 8; j++)
	{
		/* 设置光标位置为每一页的第一列 */
		OLED_SetCursor(j, 0);
		/* 顺序写入128个数据，将显存数组内容写入到OLED硬件 */
		OLED_WriteData(OLED_DisplayBuf[j], 128);
	}
}

/**
 * 函   数：将OLED显存数组部分更新到OLED屏幕
 * 参   数：X 指定区域的左上角的横坐标，范围：0~127
 * 参   数：Y 指定区域的左上角的纵坐标，范围：0~63
 * 参   数：Width 指定区域的宽度，范围：0~128
 * 参   数：Height 指定区域的高度，范围：0~64
 * 返 回 值：无
 * 说   明：仅此函数会快速更新部分指定区域
 *           但是如果更新区域Y轴只跨了两个页，则同一页的剩余部分会一起更新
 * 说   明：所有的显示函数都只是对OLED显存数组进行读写
 *           需要调用OLED_Update或OLED_UpdateArea函数
 *           才会将显存数组的数据发送到OLED硬件并触发显示
 *           故每次显示操作之后想同步到屏幕上，需要调用更新函数
 */
void OLED_UpdateArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height)
{
	uint8_t j;

	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (X > 127)
	{
		return;
	}
	if (Y > 63)
	{
		return;
	}
	if (X + Width > 128)
	{
		Width = 128 - X;
	}
	if (Y + Height > 64)
	{
		Height = 64 - Y;
	}

	/* 遍历指定区域涉及的每一页 */
	/* (Y + Height - 1) / 8 + 1的目标是(Y + Height) / 8再向上取整 */
	for (j = Y / 8; j < (Y + Height - 1) / 8 + 1; j++)
	{
		/* 设置光标位置为该页的指定列 */
		OLED_SetCursor(j, X);
		/* 顺序写入Width个数据，将显存数组内容写入到OLED硬件 */
		OLED_WriteData(&OLED_DisplayBuf[j][X], Width);
	}
}

/**
 * 函   数：将OLED显存数组全部清零
 * 参   数：无
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++) // 遍历8页
	{
		for (i = 0; i < 128; i++) // 遍历128列
		{
			OLED_DisplayBuf[j][i] = 0x00; // 将显存数组内容全部清零
		}
	}
}

/**
 * 函   数：将OLED显存数组部分清零
 * 参   数：X 指定区域的左上角的横坐标，范围：0~127
 * 参   数：Y 指定区域的左上角的纵坐标，范围：0~63
 * 参   数：Width 指定区域的宽度，范围：0~128
 * 参   数：Height 指定区域的高度，范围：0~64
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ClearArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height)
{
	uint8_t i, j;

	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (X > 127)
	{
		return;
	}
	if (Y > 63)
	{
		return;
	}
	if (X + Width > 128)
	{
		Width = 128 - X;
	}
	if (Y + Height > 64)
	{
		Height = 64 - Y;
	}

	for (j = Y; j < Y + Height; j++) // 遍历指定行
	{
		for (i = X; i < X + Width; i++) // 遍历指定列
		{
			OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8)); // 将显存数组指定像素清零
		}
	}
}

/**
 * 函   数：将OLED显存数组全部取反
 * 参   数：无
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_Reverse(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++) // 遍历8页
	{
		for (i = 0; i < 128; i++) // 遍历128列
		{
			OLED_DisplayBuf[j][i] ^= 0xFF; // 将显存数组内容全部取反
		}
	}
}

/**
 * 函   数：将OLED显存数组部分取反
 * 参   数：X 指定区域的左上角的横坐标，范围：0~127
 * 参   数：Y 指定区域的左上角的纵坐标，范围：0~63
 * 参   数：Width 指定区域的宽度，范围：0~128
 * 参   数：Height 指定区域的高度，范围：0~64
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ReverseArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height)
{
	uint8_t i, j;

	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (X > 127)
	{
		return;
	}
	if (Y > 63)
	{
		return;
	}
	if (X + Width > 128)
	{
		Width = 128 - X;
	}
	if (Y + Height > 64)
	{
		Height = 64 - Y;
	}

	for (j = Y; j < Y + Height; j++) // 遍历指定行
	{
		for (i = X; i < X + Width; i++) // 遍历指定列
		{
			OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8); // 将显存数组指定像素取反
		}
	}
}

/**
 * 函   数：OLED显示一个字符
 * 参   数：X 指定字符的左上角的横坐标，范围：0~127
 * 参   数：Y 指定字符的左上角的纵坐标，范围：0~63
 * 参   数：Char 指定要显示的字符，范围：ASCII可显字符
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowChar(uint8_t X, uint8_t Y, char Char, uint8_t FontSize)
{
	if (FontSize == OLED_8X16) // 字体为宽8像素，高16像素
	{
		/* 将ASCII字模数组OLED_F8x16的指定字符以8*16的图像格式显示 */
		OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
	}
	else if (FontSize == OLED_6X8) // 字体为宽6像素，高8像素
	{
		/* 将ASCII字模数组OLED_F6x8的指定字符以6*8的图像格式显示 */
		OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
	}
}

/**
 * 函   数：OLED显示字符串
 * 参   数：X 指定字符串的左上角的横坐标，范围：0~127
 * 参   数：Y 指定字符串的左上角的纵坐标，范围：0~63
 * 参   数：String 指定要显示的字符串，范围：ASCII可显字符组成的字符串
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowString(uint8_t X, uint8_t Y, char *String, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++) // 遍历字符串的每个字符
	{
		/* 调用OLED_ShowChar函数，依次显示每个字符 */
		OLED_ShowChar(X + i * FontSize, Y, String[i], FontSize);
	}
}

/**
 * 函   数：OLED显示数字（十进制，正整数）
 * 参   数：X 指定数字的左上角的横坐标，范围：0~127
 * 参   数：Y 指定数字的左上角的纵坐标，范围：0~63
 * 参   数：Number 指定要显示的数字，范围：0~4294967295
 * 参   数：Length 指定数字的长度，范围：0~10
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; i < Length; i++) // 遍历数字的每一位
	{
		/* 调用OLED_ShowChar函数，依次显示每个数字 */
		/* Number / OLED_Pow(10, Length - i - 1) % 10 可以十进制取出数字的每一位 */
		/* + '0' 可将数字转换为字符形式 */
		OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
}

/**
 * 函   数：OLED显示有符号数字（十进制，包含符号）
 * 参   数：X 指定数字的左上角的横坐标，范围：0~127
 * 参   数：Y 指定数字的左上角的纵坐标，范围：0~63
 * 参   数：Number 指定要显示的数字，范围：-2147483648~2147483647
 * 参   数：Length 指定数字的长度，范围：0~10
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowSignedNum(uint8_t X, uint8_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	uint32_t Number1;

	if (Number >= 0) // 数字大于等于0
	{
		OLED_ShowChar(X, Y, '+', FontSize); // 显示+号
		Number1 = Number;					// Number1直接等于Number
	}
	else // 数字小于0
	{
		OLED_ShowChar(X, Y, '-', FontSize); // 显示-号
		Number1 = -Number;					// Number1等于Number取反
	}

	for (i = 0; i < Length; i++) // 遍历数字的每一位
	{
		/* 调用OLED_ShowChar函数，依次显示每个数字 */
		/* Number1 / OLED_Pow(10, Length - i - 1) % 10 可以十进制取出数字的每一位 */
		/* + '0' 可将数字转换为字符形式 */
		OLED_ShowChar(X + (i + 1) * FontSize, Y, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
}

/**
 * 函   数：OLED显示十六进制数字（十六进制，正整数）
 * 参   数：X 指定数字的左上角的横坐标，范围：0~127
 * 参   数：Y 指定数字的左上角的纵坐标，范围：0~63
 * 参   数：Number 指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * 参   数：Length 指定数字的长度，范围：0~8
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowHexNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++) // 遍历数字的每一位
	{
		/* 以十六进制取出数字的每一位 */
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;

		if (SingleNumber < 10) // 如果数字小于10
		{
			/* 调用OLED_ShowChar函数，显示数字 */
			/* + '0' 可将数字转换为字符形式 */
			OLED_ShowChar(X + i * FontSize, Y, SingleNumber + '0', FontSize);
		}
		else // 如果数字大于10
		{
			/* 调用OLED_ShowChar函数，显示数字 */
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			OLED_ShowChar(X + i * FontSize, Y, SingleNumber - 10 + 'A', FontSize);
		}
	}
}

/**
 * 函   数：OLED显示二进制数字（二进制，正整数）
 * 参   数：X 指定数字的左上角的横坐标，范围：0~127
 * 参   数：Y 指定数字的左上角的纵坐标，范围：0~63
 * 参   数：Number 指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * 参   数：Length 指定数字的长度，范围：0~16
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowBinNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	for (i = 0; i < Length; i++) // 遍历数字的每一位
	{
		/* 调用OLED_ShowChar函数，依次显示每个数字 */
		/* Number / OLED_Pow(2, Length - i - 1) % 2 可以二进制取出数字的每一位 */
		/* + '0' 可将数字转换为字符形式 */
		OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
	}
}

/**
 * 函   数：OLED显示浮点数字（十进制，小数）
 * 参   数：X 指定数字的左上角的横坐标，范围：0~127
 * 参   数：Y 指定数字的左上角的纵坐标，范围：0~63
 * 参   数：Number 指定要显示的数字，范围：-4294967295.0~4294967295.0
 * 参   数：IntLength 指定数字的整数位长度，范围：0~10
 * 参   数：FraLength 指定数字的小数位长度，范围：0~9，过多小数位有精度丢失
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowFloatNum(uint8_t X, uint8_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
	uint32_t Temp;

	if (Number >= 0) // 数字大于等于0
	{
		OLED_ShowChar(X, Y, '+', FontSize); // 显示+号
	}
	else // 数字小于0
	{
		OLED_ShowChar(X, Y, '-', FontSize); // 显示-号
		Number = -Number;					// Number取反
	}

	/* 显示整数部分 */
	OLED_ShowNum(X + FontSize, Y, Number, IntLength, FontSize);

	/* 显示小数点 */
	OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);

	/* 把Number的整数部分减掉，以防止之后小数部分乘到整数时结果数过大 */
	Number -= (uint32_t)Number;

	/* 把小数部分乘到整数位，然后显示 */
	Temp = OLED_Pow(10, FraLength);
	OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, ((uint32_t)(Number * Temp)) % Temp, FraLength, FontSize);
}

/**
 * 函   数：OLED显示汉字串
 * 参   数：X 指定汉字串的左上角的横坐标，范围：0~127
 * 参   数：Y 指定汉字串的左上角的纵坐标，范围：0~63
 * 参   数：Chinese 指定要显示的汉字串，范围：必须全为汉字或全为ASCII字符，不要混合任意字符
 *           显示的汉字需要在OLED_Data.c中的OLED_CF16x16数组定义
 *           未找到指定汉字时，将显示默认图形（一个内部带问号的方框）
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowChinese(uint8_t X, uint8_t Y, char *Chinese)
{
	uint8_t pChinese = 0;
	uint8_t pIndex;
	uint8_t i;
	char SingleChinese[OLED_CHN_CHAR_WIDTH + 1] = {0};

	for (i = 0; Chinese[i] != '\0'; i++) // 遍历汉字串
	{
		SingleChinese[pChinese] = Chinese[i]; // 提取汉字串数据到单汉字数组
		pChinese++;							  // 计次变量

		/* 当提取量达到了OLED_CHN_CHAR_WIDTH时，即代表已经提取完一个完整的汉字 */
		if (pChinese >= OLED_CHN_CHAR_WIDTH)
		{
			pChinese = 0; // 计次归零

			/* 遍历整个汉字字模库，寻找匹配的汉字 */
			/* 如果找到了空汉字，则代表为字符串结尾，退出显示；否则未遍历字模库定义，停止寻找 */
			for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex++)
			{
				/* 找到匹配的汉字 */
				if (strcmp(OLED_CF16x16[pIndex].Index, SingleChinese) == 0)
				{
					break; // 跳出循环，此时pIndex的值为指定汉字的索引
				}
			}

			/* 将汉字字模OLED_CF16x16的指定汉字以16*16的图像格式显示 */
			OLED_ShowImage(X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 16, Y, 16, 16, OLED_CF16x16[pIndex].Data);
		}
	}
}

/**
 * 函   数：OLED显示图像
 * 参   数：X 指定图像的左上角的横坐标，范围：0~127
 * 参   数：Y 指定图像的左上角的纵坐标，范围：0~63
 * 参   数：Width 指定图像的宽度，范围：0~128
 * 参   数：Height 指定图像的高度，范围：0~64
 * 参   数：Image 指定要显示的图像
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
	uint8_t i, j;

	/* 参数检查，保证指定图像不会超出屏幕范围 */
	if (X > 127)
	{
		return;
	}
	if (Y > 63)
	{
		return;
	}

	/* 将图像区域内容清零 */
	OLED_ClearArea(X, Y, Width, Height);

	/* 遍历指定图像涉及的每一页 */
	/* (Height - 1) / 8 + 1的目标是Height / 8再向上取整 */
	for (j = 0; j < (Height - 1) / 8 + 1; j++)
	{
		/* 遍历指定图像涉及的每一列 */
		for (i = 0; i < Width; i++)
		{
			/* 超出边界，则跳过显示 */
			if (X + i > 127)
			{
				break;
			}
			if (Y / 8 + j > 7)
			{
				return;
			}

			/* 显示图像在当前页的部分 */
			OLED_DisplayBuf[Y / 8 + j][X + i] |= Image[j * Width + i] << (Y % 8);

			/* 超出边界，则跳过显示 */
			/* 使用continue的目的是：当一页超出边界时，下一页的列数据还需要继续显示 */
			if (Y / 8 + j + 1 > 7)
			{
				continue;
			}

			/* 显示图像在下一页的部分 */
			OLED_DisplayBuf[Y / 8 + j + 1][X + i] |= Image[j * Width + i] >> (8 - Y % 8);
		}
	}
}

/**
 * 函   数：OLED使用printf函数打印格式化字符串
 * 参   数：X 指定格式化字符串的左上角的横坐标，范围：0~127
 * 参   数：Y 指定格式化字符串的左上角的纵坐标，范围：0~63
 * 参   数：FontSize 指定字体大小
 *           范围：OLED_8X16       宽8像素，高16像素
 *                  OLED_6X8       宽6像素，高8像素
 * 参   数：format 指定要显示的格式化字符串，范围：ASCII可显字符组成的字符串
 * 参   数：... 格式化字符串的参数列表
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...)
{
	char String[30];						 // 定义字符数组
	va_list arg;							 // 定义可变参数列表操作类型的变量arg
	va_start(arg, format);					 // 从format开始，接收参数列表到arg中
	vsprintf(String, format, arg);			 // 使用vsprintf打印格式化字符串和参数列表到字符数组
	va_end(arg);							 // 结束参数arg
	OLED_ShowString(X, Y, String, FontSize); // OLED显示字符数组（字符串）
}

/**
 * 函   数：OLED在指定位置画一个点
 * 参   数：X 指定点的横坐标，范围：0~127
 * 参   数：Y 指定点的纵坐标，范围：0~63
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawPoint(uint8_t X, uint8_t Y)
{
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (X > 127)
	{
		return;
	}
	if (Y > 63)
	{
		return;
	}

	/* 将显存数组指定位置的一个Bit数据置1 */
	OLED_DisplayBuf[Y / 8][X] |= 0x01 << (Y % 8);
}

/**
 * 函   数：OLED获取指定位置的点的值
 * 参   数：X 指定点的横坐标，范围：0~127
 * 参   数：Y 指定点的纵坐标，范围：0~63
 * 返 回 值：指定位置的点是否在点亮状态：1点亮，0熄灭
 */
uint8_t OLED_GetPoint(uint8_t X, uint8_t Y)
{
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (X > 127)
	{
		return 0;
	}
	if (Y > 63)
	{
		return 0;
	}

	/* 判断指定位置的点亮灭 */
	if (OLED_DisplayBuf[Y / 8][X] & 0x01 << (Y % 8))
	{
		return 1; // 为1，返回1
	}

	return 0; // 否则，返回0
}

/**
 * 函   数：OLED画线
 * 参   数：X0 指定一个端点的横坐标，范围：0~127
 * 参   数：Y0 指定一个端点的纵坐标，范围：0~63
 * 参   数：X1 指定另一个端点的横坐标，范围：0~127
 * 参   数：Y1 指定另一个端点的纵坐标，范围：0~63
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1)
{
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
	uint8_t yflag = 0, xyflag = 0;

	if (y0 == y1) // 横线的情况
	{
		/* 0号点的X坐标大于1号点的X坐标，则交换两点X坐标 */
		if (x0 > x1)
		{
			temp = x0;
			x0 = x1;
			x1 = temp;
		}

		/* 遍历X坐标 */
		for (x = x0; x <= x1; x++)
		{
			OLED_DrawPoint(x, y0); // 依次画点
		}
	}
	else if (x0 == x1) // 横线的情况
	{
		/* 0号点的Y坐标大于1号点的Y坐标，则交换两点Y坐标 */
		if (y0 > y1)
		{
			temp = y0;
			y0 = y1;
			y1 = temp;
		}

		/* 遍历Y坐标 */
		for (y = y0; y <= y1; y++)
		{
			OLED_DrawPoint(x0, y); // 依次画点
		}
	}
	else // 斜线
	{
		/* 使用Bresenham算法画直线，可以避免耗时的高等数学计算，效率更高 */
		/* 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf */
		/* 参考教程：https://www.bilibili.com/video/BV1364y1d7Lo */

		if (x0 > x1) // 0号点的X坐标大于1号点的X坐标
		{
			/* 交换两点坐标 */
			/* 交换不影响画线，但是画线方向由第一 / 三象限变为第二 / 四象限 */
			temp = x0;
			x0 = x1;
			x1 = temp;
			temp = y0;
			y0 = y1;
			y1 = temp;
		}

		if (y0 > y1) // 0号点的Y坐标大于1号点的Y坐标
		{
			/* 将Y坐标取反 */
			/* 取反不影响画线，但是画线方向由第一 / 三象限变为第一 / 四象限 */
			y0 = -y0;
			y1 = -y1;

			/* 用标志位yflag记住当前变换，在后续实际画点时，再将坐标变换回来 */
			yflag = 1;
		}

		if (y1 - y0 > x1 - x0) // 如果斜率大于1
		{
			/* 将X坐标与Y坐标互换 */
			/* 交换不影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围 */
			temp = x0;
			x0 = y0;
			y0 = temp;
			temp = x1;
			x1 = y1;
			y1 = temp;

			/* 用标志位xyflag记住当前变换，在后续实际画点时，再将坐标变换回来 */
			xyflag = 1;
		}

		/* 以下为Bresenham算法画直线 */
		/* 算法要求，画线方向必须为第一象限0~45度范围 */
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;

		/* 画起始点，同时判断标志位进行坐标变换 */
		if (yflag && xyflag)
		{
			OLED_DrawPoint(y, -x);
		}
		else if (yflag)
		{
			OLED_DrawPoint(x, -y);
		}
		else if (xyflag)
		{
			OLED_DrawPoint(y, x);
		}
		else
		{
			OLED_DrawPoint(x, y);
		}

		while (x < x1) // 遍历X轴的每个点
		{
			x++;
			if (d < 0) // 下一点在当前位置的东面
			{
				d += incrE;
			}
			else // 下一点在当前位置的东面
			{
				y++;
				d += incrNE;
			}

			/* 画每一个点，同时判断标志位进行坐标变换 */
			if (yflag && xyflag)
			{
				OLED_DrawPoint(y, -x);
			}
			else if (yflag)
			{
				OLED_DrawPoint(x, -y);
			}
			else if (xyflag)
			{
				OLED_DrawPoint(y, x);
			}
			else
			{
				OLED_DrawPoint(x, y);
			}
		}
	}
}

/**
 * 函   数：OLED画矩形
 * 参   数：X 指定矩形的左上角的横坐标，范围：0~127
 * 参   数：Y 指定矩形的左上角的纵坐标，范围：0~63
 * 参   数：Width 指定矩形的宽度，范围：0~128
 * 参   数：Height 指定矩形的高度，范围：0~64
 * 参   数：IsFilled 指定矩形是否填充
 *           范围：OLED_UNFILLED        不填充
 *                  OLED_FILLED          填充
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawRectangle(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
	uint8_t i, j;
	if (!IsFilled) // 指定矩形不填充
	{
		/* 遍历宽度X坐标，画上下两条横线 */
		for (i = X; i < X + Width; i++)
		{
			OLED_DrawPoint(i, Y);
			OLED_DrawPoint(i, Y + Height - 1);
		}
		/* 遍历高度Y坐标，画左右两条竖线 */
		for (i = Y; i < Y + Height; i++)
		{
			OLED_DrawPoint(X, i);
			OLED_DrawPoint(X + Width - 1, i);
		}
	}
	else // 指定矩形填充
	{
		/* 遍历X坐标 */
		for (i = X; i < X + Width; i++)
		{
			/* 遍历Y坐标 */
			for (j = Y; j < Y + Height; j++)
			{
				/* 在指定区域画点，填充满矩形 */
				OLED_DrawPoint(i, j);
			}
		}
	}
}

/**
 * 函   数：OLED画三角形
 * 参   数：X0 指定第一个端点的横坐标，范围：0~127
 * 参   数：Y0 指定第一个端点的纵坐标，范围：0~63
 * 参   数：X1 指定第二个端点的横坐标，范围：0~127
 * 参   数：Y1 指定第二个端点的纵坐标，范围：0~63
 * 参   数：X2 指定第三个端点的横坐标，范围：0~127
 * 参   数：Y2 指定第三个端点的纵坐标，范围：0~63
 * 参   数：IsFilled 指定三角形是否填充
 *           范围：OLED_UNFILLED        不填充
 *                  OLED_FILLED          填充
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawTriangle(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t IsFilled)
{
	uint8_t minx = X0, miny = Y0, maxx = X0, maxy = Y0;
	uint8_t i, j;
	int16_t vx[] = {X0, X1, X2};
	int16_t vy[] = {Y0, Y1, Y2};

	if (!IsFilled) // 指定三角形不填充
	{
		/* 调用画线函数，画出三条直线边缘 */
		OLED_DrawLine(X0, Y0, X1, Y1);
		OLED_DrawLine(X0, Y0, X2, Y2);
		OLED_DrawLine(X1, Y1, X2, Y2);
	}
	else // 指定矩形填充
	{
		/* 找到三角形的最小X、Y坐标 */
		if (X1 < minx)
		{
			minx = X1;
		}
		if (X2 < minx)
		{
			minx = X2;
		}
		if (Y1 < miny)
		{
			miny = Y1;
		}
		if (Y2 < miny)
		{
			miny = Y2;
		}

		/* 找到三角形的最大X、Y坐标 */
		if (X1 > maxx)
		{
			maxx = X1;
		}
		if (X2 > maxx)
		{
			maxx = X2;
		}
		if (Y1 > maxy)
		{
			maxy = Y1;
		}
		if (Y2 > maxy)
		{
			maxy = Y2;
		}

		/* 以最小坐标到最大坐标的距离为矩形，遍历整个矩形 */
		/* 遍历矩形中所有的点 */
		/* 遍历X坐标 */
		for (i = minx; i <= maxx; i++)
		{
			/* 遍历Y坐标 */
			for (j = miny; j <= maxy; j++)
			{
				/* 使用OLED_pnpoly，判断指定点是否在指定三角形之内 */
				/* 如果在，则画点，如果不在，则不做处理 */
				if (OLED_pnpoly(3, vx, vy, i, j))
				{
					OLED_DrawPoint(i, j);
				}
			}
		}
	}
}

/**
 * 函   数：OLED画圆
 * 参   数：X 指定圆的圆心横坐标，范围：0~127
 * 参   数：Y 指定圆的圆心纵坐标，范围：0~63
 * 参   数：Radius 指定圆的半径，范围：0~255
 * 参   数：IsFilled 指定圆是否填充
 *           范围：OLED_UNFILLED        不填充
 *                  OLED_FILLED          填充
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	/* 使用Bresenham算法画圆，可以避免耗时的高等数学计算，效率更高 */
	/* 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf */
	/* 参考教程：https://www.bilibili.com/video/BV1VM4y1u7wJ */

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/* 画每个八分之一圆弧的起始点 */
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X + y, Y + x);
	OLED_DrawPoint(X - y, Y - x);

	if (IsFilled) // 指定圆填充
	{
		/* 遍历起始Y坐标 */
		for (j = -y; j < y; j++)
		{
			/* 在指定区域画点，填充部分圆 */
			OLED_DrawPoint(X, Y + j);
		}
	}

	while (x < y) // 遍历X轴的每个点
	{
		x++;
		if (d < 0) // 下一点在当前位置的东面
		{
			d += 2 * x + 1;
		}
		else // 下一点在当前位置的东南面
		{
			y--;
			d += 2 * (x - y) + 1;
		}

		/* 画每个八分之一圆弧的点 */
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X + y, Y + x);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - y, Y - x);
		OLED_DrawPoint(X + x, Y - y);
		OLED_DrawPoint(X + y, Y - x);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X - y, Y + x);

		if (IsFilled) // 指定圆填充
		{
			/* 遍历中间部分 */
			for (j = -y; j < y; j++)
			{
				/* 在指定区域画点，填充部分圆 */
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}

			/* 遍历两侧部分 */
			for (j = -x; j < x; j++)
			{
				/* 在指定区域画点，填充部分圆 */
				OLED_DrawPoint(X - y, Y + j);
				OLED_DrawPoint(X + y, Y + j);
			}
		}
	}
}

/**
 * 函   数：OLED画椭圆
 * 参   数：X 指定椭圆的圆心横坐标，范围：0~127
 * 参   数：Y 指定椭圆的圆心纵坐标，范围：0~63
 * 参   数：A 指定椭圆的横向轴长度，范围：0~255
 * 参   数：B 指定椭圆的纵向轴长度，范围：0~255
 * 参   数：IsFilled 指定椭圆是否填充
 *           范围：OLED_UNFILLED        不填充
 *                  OLED_FILLED          填充
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawEllipse(uint8_t X, uint8_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
	int16_t x, y, j;
	int16_t a = A, b = B;
	float d1, d2;

	/* 使用Bresenham算法画椭圆，可以避免部分耗时的高等数学计算，效率更高 */
	/* 参考链接：https://blog.csdn.net/myf_666/article/details/128167392 */

	x = 0;
	y = b;
	d1 = b * b + a * a * (-b + 0.5);

	if (IsFilled) // 指定椭圆填充
	{
		/* 遍历起始Y坐标 */
		for (j = -y; j < y; j++)
		{
			/* 在指定区域画点，填充部分椭圆 */
			OLED_DrawPoint(X, Y + j);
			OLED_DrawPoint(X, Y + j);
		}
	}

	/* 画椭圆上部起始点 */
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X - x, Y + y);
	OLED_DrawPoint(X + x, Y - y);

	/* 画椭圆中间部分 */
	while (b * b * (x + 1) < a * a * (y - 0.5))
	{
		if (d1 <= 0) // 下一点在当前位置的东面
		{
			d1 += b * b * (2 * x + 3);
		}
		else // 下一点在当前位置的东南面
		{
			d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
			y--;
		}
		x++;

		if (IsFilled) // 指定椭圆填充
		{
			/* 遍历中间部分 */
			for (j = -y; j < y; j++)
			{
				/* 在指定区域画点，填充部分椭圆 */
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}

		/* 画椭圆中间部分圆弧 */
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}

	/* 画椭圆两侧部分 */
	d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;

	while (y > 0)
	{
		if (d2 <= 0) // 下一点在当前位置的东面
		{
			d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
			x++;
		}
		else // 下一点在当前位置的东南面
		{
			d2 += a * a * (-2 * y + 3);
		}
		y--;

		if (IsFilled) // 指定椭圆填充
		{
			/* 遍历两侧部分 */
			for (j = -y; j < y; j++)
			{
				/* 在指定区域画点，填充部分椭圆 */
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}

		/* 画椭圆两侧部分圆弧 */
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}
}

/**
 * 函   数：OLED画圆弧
 * 参   数：X 指定圆弧的圆心横坐标，范围：0~127
 * 参   数：Y 指定圆弧的圆心纵坐标，范围：0~63
 * 参   数：Radius 指定圆弧的半径，范围：0~255
 * 参   数：StartAngle 指定圆弧的起始角度，范围：-180~180
 *           水平向右为0度，水平向左为180度或-180度，下方为正，上方为负，顺时针旋转
 * 参   数：EndAngle 指定圆弧的终止角度，范围：-180~180
 *           水平向右为0度，水平向左为180度或-180度，下方为正，上方为负，顺时针旋转
 * 参   数：IsFilled 指定圆弧是否填充，填充为扇形
 *           范围：OLED_UNFILLED        不填充
 *                  OLED_FILLED          填充
 * 返 回 值：无
 * 说   明：调用此函数后需要同步到屏幕上，需要调用更新函数
 */
void OLED_DrawArc(uint8_t X, uint8_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
	int16_t x, y, d, j;

	/* 此函数基于Bresenham算法画圆的方法 */

	d = 1 - Radius;
	x = 0;
	y = Radius;

	/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
	if (OLED_IsInAngle(x, y, StartAngle, EndAngle))
	{
		OLED_DrawPoint(X + x, Y + y);
	}
	if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle))
	{
		OLED_DrawPoint(X - x, Y - y);
	}
	if (OLED_IsInAngle(y, x, StartAngle, EndAngle))
	{
		OLED_DrawPoint(X + y, Y + x);
	}
	if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle))
	{
		OLED_DrawPoint(X - y, Y - x);
	}

	if (IsFilled) // 指定圆填充
	{
		/* 遍历起始Y坐标 */
		for (j = -y; j < y; j++)
		{
			/* 在画圆的每个点时，判断指定点是否在指定角度内，在则画点，不在则不做处理 */
			if (OLED_IsInAngle(0, j, StartAngle, EndAngle))
			{
				OLED_DrawPoint(X, Y + j);
			}
		}
	}

	while (x < y) // 遍历X轴的每个点
	{
		x++;
		if (d < 0) // 下一点在当前位置的东面
		{
			d += 2 * x + 1;
		}
		else // 下一点在当前位置的东南面
		{
			y--;
			d += 2 * (x - y) + 1;
		}

		/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
		if (OLED_IsInAngle(x, y, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X + x, Y + y);
		}
		if (OLED_IsInAngle(y, x, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X + y, Y + x);
		}
		if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X - x, Y - y);
		}
		if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X - y, Y - x);
		}
		if (OLED_IsInAngle(x, -y, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X + x, Y - y);
		}
		if (OLED_IsInAngle(y, -x, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X + y, Y - x);
		}
		if (OLED_IsInAngle(-x, y, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X - x, Y + y);
		}
		if (OLED_IsInAngle(-y, x, StartAngle, EndAngle))
		{
			OLED_DrawPoint(X - y, Y + x);
		}

		if (IsFilled) // 指定圆填充
		{
			/* 遍历中间部分 */
			for (j = -y; j < y; j++)
			{
				/* 在画圆的每个点时，判断指定点是否在指定角度内，在则画点，不在则不做处理 */
				if (OLED_IsInAngle(x, j, StartAngle, EndAngle))
				{
					OLED_DrawPoint(X + x, Y + j);
				}
				if (OLED_IsInAngle(-x, j, StartAngle, EndAngle))
				{
					OLED_DrawPoint(X - x, Y + j);
				}
			}

			/* 遍历两侧部分 */
			for (j = -x; j < x; j++)
			{
				/* 在画圆的每个点时，判断指定点是否在指定角度内，在则画点，不在则不做处理 */
				if (OLED_IsInAngle(-y, j, StartAngle, EndAngle))
				{
					OLED_DrawPoint(X - y, Y + j);
				}
				if (OLED_IsInAngle(y, j, StartAngle, EndAngle))
				{
					OLED_DrawPoint(X + y, Y + j);
				}
			}
		}
	}
}

/********************* 显示函数 */

/***************** 江协科技|版权所有 ****************/
/***************** jiangxiekeji.com *****************/
