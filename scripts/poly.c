#include "stdio.h"
#include "math.h"

#define rows 16
#define cols 16

FILE* fp;

double poly(double x, double y){
	return pow(x,3) - 10*pow(x,2) - pow(x,2) + 1;
}

int main(int argc, char** argv){
	printf("Hello Python\n");

	fp = fopen("./data.txt", "r");

	double xy_min = -5.0;
	double xy_max = 5.0;
	double n = 16;


	for (int i = 0; i<rows; i++){
		for(int j = 0;j<cols; j++){
			printf("%.2lf\t", poly( ((double)(xy_min + i*n/(xy_max-xy_min))),((double) (xy_min + j*n/(xy_max-xy_min)))));
		}
	}



	return 0;
}