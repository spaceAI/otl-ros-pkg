//
//  OTLI-SOBOTIF.m
//  i-gui
//
//  Created by 小倉 崇 on 09/11/13.
//  Copyright 2009 OTL. All rights reserved.
//

#import "OTLI-SOBOTIF.h"

@interface OTLI_SOBOTIF (private)
- (BOOL)setJointAnglesInternal:(double *)ang time:(double)tm;
@end

@implementation OTLI_SOBOTIF

- (void)setSerialPort
{
	struct termios newtio;
	tcgetattr(fd, &oldtio);
	memset(&newtio,0,sizeof(newtio));
	
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD ;
    newtio.c_iflag = 0;
    newtio.c_cc[VTIME] = 0;
//    newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VMIN] = 0;
	// set speed
   	cfsetospeed(&newtio, B9600);
	cfsetispeed(&newtio, B9600);

    tcflush(fd, TCIFLUSH);

    // 新しい設定を適用
//   tcsetattr(fd, TCSANOW,&newtio);
	if ( tcsetattr(fd, TCSAFLUSH, &newtio) != 0 )
	{
		NSLog(@"fail to set serial port");
	}
	
}

- (void)setInitialAngles
{

	offset[0] = -15;
	offset[1] = 5;
	offset[2] = -20;
	offset[3] = -5;
	
	
	angles[0] = 0;
	
	angles[1] = 0;
	angles[2] = 0;
	angles[3] = 0;
	
	angles[4] = 0;
	angles[5] =0;

	// servo on
	for (int i = 0; i < I_SOBOT_DOF; i++)
	{
		servo[i] = 1;
	}
	// -50 0 0 kamae
	// 100 -75 100 open
	// 100 0 100 pre-hold
	// 100 0 0 hold
	// -64 0 0 forward
	// -64 -90 0 sosogi
	// -64 0 0 modosu
	// set limit angles (High)

	loAngleLimits[0] =  -80;
	hiAngleLimits[0] =  100;
	
	loAngleLimits[1] = -100;
	hiAngleLimits[1] =  100;

	loAngleLimits[2] = -92;
	hiAngleLimits[2] =  92;

	loAngleLimits[3] = -35;
	hiAngleLimits[3] =  35;
	
	loAngleLimits[4] = -100;
	hiAngleLimits[4] =  100;

	loAngleLimits[5] = -92;
	hiAngleLimits[5] =  92;

}


// シリアルデバイスをオープンして初期化
- (id)initWithDeviceName:(NSString *)name
{
	// stringを文字列に変換
	const char *devName = [name UTF8String];

	// open serial (read & write)
	fd = open(devName, O_RDWR | O_NOCTTY);

	// fail to open
	if (fd < 0)
	{
		NSLog(@"failed to open %@", name);
		return nil;
	}
	
	[self setSerialPort];
	NSLog(@"opened %@", name);
	
	return [self init];
	
}

// initialize without open device
- (id)init
{
	double ang[I_SOBOT_DOF];
	
	[super init];
	[self setInitialAngles];
	[self getJointAngles:ang];
	[self setJointAnglesInternal:ang time:100];
	
	return self;	
}

- (BOOL)angleLimitFilter:(double *)ang at:(int)i
{
	BOOL ret = NO;
	if ( *ang > hiAngleLimits[i] )
	{
		*ang = hiAngleLimits[i];
		ret = YES;
	}
	else if ( *ang < loAngleLimits[i] )
	{
		*ang = loAngleLimits[i];
		ret = YES;
	}
	return ret;
}

- (BOOL)checkAngleLimits:(double *)ang
{
	BOOL ret = NO;
	
	for (int i = 0; i < I_SOBOT_DOF; i++)
	{
		if ([self angleLimitFilter:&ang[i] at:i])
		{
			ret = YES;
		}
	}
	return ret;
}

- (BOOL)setJointAngles:(double *)ang time:(double)tm
{
	[self checkAngleLimits:ang];
	return ([self setJointAnglesInternal:ang time:tm]);

}

- (BOOL)setJointAngle:(double)ang at:(int)i time:(double)tm
{
	double tang[I_SOBOT_DOF];
	[self getJointAngles:tang];
	tang[i] = ang;
	[self checkAngleLimits:tang];
	return ([self setJointAnglesInternal:tang time:tm]);
}

- (BOOL)setJointAnglesInternal:(double *)ang time:(double)tm
{
	NSString *buf = @"a";
	int wroteByte = 0;
	int tangles[I_SOBOT_DOF]; // 書き込み用バッファ
	
	for (int i = 0; i < I_SOBOT_DOF; i++)
	{
		// 角度の保存
		angles[i] = (double)ang[i];

		if ( servo[i] == 0 )
		{
			// tanglesは 0だとサーボOFF
			tangles[i] = 0;
		}
		else
		{
			// tanglesは 1-255
			tangles[i] = (int)(((int)ang[i] + offset[i] + 135) * 180 / 270);
			if (tangles[i] > 180)
			{
				tangles[i] = 180;
			}else if (tangles[i] <= 0)
			{
				tangles[i] = 0;
			}
		}
	}
	for (int i = 0; i < I_SOBOT_DOF-1; i++)
	{
		buf = [buf stringByAppendingFormat:@"%d,", tangles[i]];
	}
	buf = [buf stringByAppendingFormat:@"%dE", tangles[I_SOBOT_DOF-1]];

	const char *cbuf = [buf UTF8String];
	char rbuf[255];
	int len = [buf length];
	int ret = 0;

	if ( fd > 0)
	{
		while(wroteByte < len)
		{
			ret = write(fd, &(cbuf[wroteByte]), len- wroteByte);
			if (ret > 0)
			{
				wroteByte += ret;
			}
			else
			{
				NSLog(@"failed to write");
				return NO;
			}
		}
		read(fd, &(rbuf[0]), 255);
		if ( cbuf[0] == 'a' )
		{
			NSLog(@"success to read");
		}
		else {
			NSLog(@"responce error! %s", rbuf);
		}

		
	}
	
	NSLog(@"wrote %@", buf);
	return YES;
}


- (BOOL)getJointAngle:(double *)ang at:(int)i
{
	double tang[I_SOBOT_DOF];	
	[self getJointAngles:tang];
	*ang = tang[i];
	
	return YES;
}

- (BOOL)getJointAngles:(double *)ang
{
	for (int i = 0; i < I_SOBOT_DOF; i++)
	{
		ang[i] = angles[i];
	}
	return YES;
}

- (BOOL)setJointServo:(int)onoff at:(int)i
{
	double ang[I_SOBOT_DOF];

	servo[i] = onoff;
	// 反映
	[self getJointAngles:ang];
	[self setJointAnglesInternal:ang time:100];
	
	return YES;
}

- (BOOL)getJointServo:(int *)onoff at:(int)i
{
	*onoff = servo[i];
	return YES;
}

- (BOOL)setJointServos:(int *)onoff
{
	double ang[I_SOBOT_DOF];
	
	memcpy(servo, onoff, sizeof(int)*I_SOBOT_DOF);
	// 反映
	[self getJointAngles:ang];
	[self setJointAngles:ang time:100];
	
	return YES;
}

- (BOOL)getJointServos:(int *)onoff
{
	memcpy(onoff, servo, sizeof(int)*I_SOBOT_DOF);
	return YES;
}

- (void)dealloc
{
	// 設定を元に戻す
	tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
	NSLog(@"closed device");
	[super dealloc];	
}


@end
