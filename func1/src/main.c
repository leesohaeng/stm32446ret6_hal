/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Distribution: The file is distributed "as is", without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

#include<stdio.h>
#include<math.h>     // 수학함수를 사용하기 위해...
// 사용자 함수 만들기

// 사용자함수의 원형을 선언한다.
// 컴파일러에게 사용자가 정의한 함수가 있음을 알려주는 역할.
int      f(int x, int y);
double   rf(double a, double b);

// 사용자 함수를 구현한다.
int f(int x, int y)
{
	int r;

	// 계산된 결과는 r에 담는다.
	r = 3*x*x*x*y*y + 5*x*y*y - 7*x*x*y - 15;

	return r;
}

double rf(double a, double b)
{
	double r;

	r = sqrt(a*b) / sqrt(a);
	return r;
}

int main(void){
	int x;

	printf("Result : %d\n",f(1,1));
	printf("Result : %d\n",f(1,2));
	printf("Result : %d\n",f(4,7));

	printf("Result : %d\n",f(2,3)*f(3,2) ) ;

	printf("*Result : %f\n",rf(3,5));



	return 0;
}
