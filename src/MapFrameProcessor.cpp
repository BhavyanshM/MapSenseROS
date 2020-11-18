#include <opencv2/highgui.hpp>
#include "MapFrameProcessor.h"

void MapFrameProcessor::generateSegmentation(MapFrame frame) {
    // this->frame = frame;
    int components = 1;
    vector<int> sizes;
    vector<int> ids;
    Mat debug(600,800, CV_8UC3);
    debug = Scalar(0);
    for (int i = 0; i < frame.SUB_H; i++) {
        for (int j = 0; j < frame.SUB_W; j++) {
            uint8_t patch = frame.getPatchData().at<uint8_t>(i,j);
            if(patch == 255) printf("\nFound(%d,%d,%d):", i, j, patch);
            if (!visited[i][j] && patch == 255) {
                printf("DFS(%d,%d,%d)->", i,j,patch);

                int num = dfs(i, j, components, 0, debug);
                if (num != 0){
                    components++;
                    sizes.push_back(num);
                    ids.push_back(components);
                }

            }
            // else{
            //     printf("\nNot Found DFS(%d,%d,%d)\n", i,j , visited[i][j]);
            // }
        }
    }
    // cout << visited << endl;

    for(int m = 0; m<60; m++){
        for(int n = 0; n<80; n++){
            int patchData = region[m][n];

            if (patchData < 10){
                printf("%d ",patchData);
            }else{
                printf("0 ");
            }
        }
        printf("\n");
    }
    printf("\n");

    printf("[");
    for(int k = 0; k< sizes.size(); k++){
        printf("(%d,%d), ", sizes[k], ids[k]);
    }
    printf("]\n");
    printf("Components:%d\n", components);
}


int MapFrameProcessor::dfs(int x, int y, int component, int n, Mat& debug) {
    if (visited[x][y]) return 0;

    visited[x][y] = true;
    region[x][y] = component;

    int count = 0;
    // circle(debug, Point((y)*10, (x)*10), 2, Scalar(0,120,0), -1);
    circle(debug, Point((y)*10, (x)*10), 2, Scalar(component*231 % 255,component*123 % 255,component*312 % 255), -1);
    imshow("Debug", debug);
    if(waitKeyEx(1) == 1048689) exit(1);

    for (int i = 0; i < 8; i++) {
        if (x + adx[i] < frame.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < frame.SUB_W - 1 && y + ady[i] > 1) {

            // circle(debug, Point((y + ady[i])*10, (x + adx[i])*10), 2, Scalar(component*231 % 255,component*123 % 255,component*312 % 255), -1);

            uint8_t patch = frame.getPatchData().at<uint8_t>(x + adx[i], y + ady[i]);
            printf(" LookUp(%d,%d,%d)->", adx[i], ady[i], patch);
            if (patch > 253){

                printf("DFSi(%d,%d,%d)->", x+adx[i], y+ady[i],patch);
                printf(" Total:(%d) ", n);
                count += dfs(x+adx[i], y+ady[i], component, n+1, debug);
                printf("\nCOUNT=%d\n",count);
            }
        }
    }
    return n + count;
}