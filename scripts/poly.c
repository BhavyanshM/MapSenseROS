#include "stdio.h"
#include "math.h"

#define rows 16
#define cols 16

FILE* fp;

double poly(double x, double y){
	return pow(y,3) - 10*pow(x,2) - pow(y,2) + 1;
}

int main(int argc, char** argv){
	printf("Hello Python\n");

	fp = fopen("./data.txt", "r");

	double xy_min = -5.0;
	double xy_max = 5.0;
	double n = 15;
	double result = 0;

	for (int i = 0; i<rows; i++){
		for(int j = 0;j<cols; j++){
			result = poly( ((double)(xy_min + i*(xy_max-xy_min)/n)),((double) (xy_min + j*(xy_max-xy_min)/n)));
			// printf("%.2lf\t%.2lf\n", (xy_min + i*(xy_max-xy_min)/n), (xy_min + j*(xy_max-xy_min)/n));
			printf("%.2lf\t",result);
		}
		printf("\n");
	}



	return 0;
}