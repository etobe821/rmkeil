# LOG

## 4.13

### **1、**PID

```c
第一版pid
void PIDcalculate(PidController *pid[],int i)
{
	pid->error[1]=(fp32)((pid->point_now[1])-(pid->target[1]));//实际上是target-now
	pid->errorI[1]=(pid->previoua_error[1])+(pid->error[1]);//错了
	pid->errorD[1]=(pid->error[1])-(pid->previoua_error[1]);//错了
	pid->previoua_error[1]=pid->error[1];
	pid->pidOutput[1]=pid->Kp*pid->error[1]+pid->Ki*pid->errorI[1]+pid->Kd*pid->errorD[1];
	
	pid->error[2]=(fp32)((pid->point_now[2])-(pid->target[2]));
	pid->errorI[2]=(pid->previoua_error[2])+(pid->error[2]);
	pid->errorD[2]=(pid->error[2])-(pid->previoua_error[2]);
	pid->previoua_error[2]=pid->error[2];
	pid->pidOutput[2]=pid->Kp*pid->error[2]+pid->Ki*pid->errorI[2]+pid->Kd*pid->errorD[2];
	
	pid->error[3]=(fp32)((pid->point_now[3])-(pid->target[3]));
	pid->errorI[3]=(pid->previoua_error[3])+(pid->error[3]);
	pid->errorD[3]=(pid->error[3])-(pid->previoua_error[3]);
	pid->previoua_error[3]=pid->error[3];
	pid->pidOutput[3]=pid->Kp*pid->error[3]+pid->Ki*pid->errorI[3]+pid->Kd*pid->errorD[3];
	
}

第二版

void pidData(motor_data temp[],PidController pid[],int i)
{
	pid[i].point_now=temp[i].ro_speed;
	pid[i].error=(fp32)((pid[i].point_now)-(pid[i].target));
	pid[i].errorI=(pid[i].previoua_error)+(pid[i].error);
	pid[i].errorD=(pid[i].error)-(pid[i].previoua_error);
	pid[i].previoua_error=pid[i].error;
	pid[i].pidOutput=pid[i].Kp*pid[i].error+pid[i].Ki*pid[i].errorI+pid[i].Kd*pid[i].errorD;
	
}
```

这样写函数的利用率不高，而且pid[i]只能对应error[i]，这样一个对象的其他error就没有用了



```c
fp32 PidCalculate(PidController *pid,fp32 data_now,fp32 target)
{
	pid->error=-data_now+target;
	pid->errorI+=pid->error;
	pid->errorD=-pid->error+pid->previous_error;
	pid->pidOutput=pid->Kp*pid->error+pid->errorI*pid->Ki+pid->errorD*pid->Kd;
	pid->previous_error=pid->error;
	return pid->pidOutput;//写积分限幅
	
}
```

这样写效果好，要抽象概括





```c
#define INITIALIZE_PIDCONTROLLER \
    { \
        .Kp = 10.0f, \
        .Ki = 0.1f, \
        .Kd = 0.0f, \
        .previous_error = 0.0f, \
        .error = 0.0f, \
        .errorI = 0.0f, \
        .errorD = 0.0f, \
        .pidOutput = 0.0f \
    }
```

结构体赋初始值用宏定义，加上斜杠是为了说明是同一行



### 2、代码结构

```c
void PidDataCalculate(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[])
{
	pid[3].target=spd->tagert4;//target就是麦轮解算，应该要单独写出来，如果直接放入麦轮解算的式子意义不明
  pidData(&temp[3],pid,3);
    //这两行都是pid的计算，这样子就复杂了
	motorn->speed_4=(short) (pid[3].pidOutput);
	
	pid[0].target=spd->tagert1;
  pidData(&temp[0],pid,0);
	motorn->speed_1=pid[0].pidOutput;
	
	pid[1].target=spd->tagert2;
  pidData(&temp[1],pid,1);
	motorn->speed_2=pid[1].pidOutput;
	
	pid[2].target=spd->tagert3;
  pidData(&temp[2],pid,2);
  motorn->speed_3=pid[2].pidOutput;
}
//这个函数只能针对底盘上的电机，归根结底还是pid函数写的不好
void controlspeed(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[])
{
//	wheelSpeedSolute(motorn,re);
	PIDcalculate(motorn,&temp,spd,&pid);
	speed_200(motorn);
}//之前不知道能在.c文件里面写whhile，所以就写了很多函数
```

同一个地方要实现的功能要放在一起





```c
PidController pid[4];
void test(void)
{
	for (int i=0;i<=3;i++)
	{
		speed[i]=PidCalculate(&pid[i],&temp[i],&target[i]);
	}

}

void classistask(void)
{
	while(1)
	{
		wheelSpeedSolute(&spd,&re1);
		Pid_classis_speed_calculate();//可以用来替代这个
		speed_200(&classis_motor);
	  HAL_Delay(2);
	   
	}


}
```





### 3、pid调参

**stmstudio**

stlink接上后要按复位键才可以动态地调pid





### **4**

```c
char *a[4];//可以存四个字符串，不过是字符串常量
char a[4];//存四个字符
char s[]="da";//应该用字符串数组

struct book{};
typedef struct{}typename;//是为了减少写struct book name;=>typename name

const int *a;//常量指针，量不变
int *const a;//指针常量，指针地址不变

union,enum,volatile



#include <iostream>
#include <cstring> // 使用 <cstring> 而不是 <string.h>，更符合 C++ 的规范
using namespace std;

int main() {
    char str[] = "abAajk"; // 使用字符数组而不是字符串常量
    int len = strlen(str); // 获取字符串长度
    int i = 0, j = 0;

    while (i < len) { // 外层循环遍历字符串
        if (str[i] >= 'A' && str[i] <= 'Z') { // 判断是否为大写字母
            j++;
            for (int k = i; k < len - j; k++) { // 内层循环删除大写字母
                str[k] = str[k + 1];
            }
        }
        else {
            i++; // 如果不是大写字母，继续遍历下一个字符
        }
    }

    str[len - j] = '\0'; // 修复字符串的结尾
    cout << str << endl; // 输出结果

    return 0;
}
```

