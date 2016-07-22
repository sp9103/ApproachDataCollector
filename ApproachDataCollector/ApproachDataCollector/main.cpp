#include <stdio.h>

#include "KinectMangerThread.h"
#include "ColorBasedTracker.h"

#define DATAPATH "D:\\ApproachData"

void CreateDatadir(const char* dir);
void writeDepthData(cv::Mat src, char* path, char* name);

int main(){
	KinectMangerThread kinectManager;
	ColorBasedTracker tracker;

	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	char className[256], dataDir[256];
	cv::Mat rgbBack, depthBack, pcBack;

	kinectManager.Initialize(RobotROI);

	printf("Enter class name.\n");
	scanf("%s", className);
	sprintf(dataDir, "%s\\%s", DATAPATH, className);
	CreateDatadir(dataDir);
	//background write
	char backPath[256];
	sprintf(backPath, "%s\\background.bmp", dataDir);
	rgbBack = kinectManager.getImg();
	depthBack = kinectManager.getDepth();
	pcBack = kinectManager.getPointCloud();
	tracker.InsertBackGround(rgbBack, depthBack);		//tracker initialize
	imwrite(backPath, rgbBack);							//rgb 배경저장
	writeDepthData(depthBack, dataDir, "background");

	//kinectManager.get

	while(!kinectManager.isThreadDead()){
		
	}

	kinectManager.Deinitialize();

	return 0;
}

void CreateDatadir(const char* dir){
	TCHAR szDir[MAX_PATH] = {0, };
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR pointcloudDir[MAX_PATH] = {0,};
	TCHAR approachDir[MAX_PATH] = {0,};
	char dirpath[256];
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dir, strlen(dir), szDir, MAX_PATH);
	bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
	sprintf(dirpath, "%s\\RGB", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리 - 원본
	sprintf(dirpath, "%s\\DEPTH", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리 - 원본
	sprintf(dirpath, "%s\\POINTCLOUD", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), pointcloudDir, MAX_PATH);
	mkdir_check = CreateDirectory(pointcloudDir, NULL);											//point cloud dir - 원본
	sprintf(dirpath, "%s\\APPROACH", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), approachDir, MAX_PATH);
	mkdir_check = CreateDirectory(approachDir, NULL);											//approaching point dir
}

void writeDepthData(cv::Mat src, char* path, char* name){
	//Depth Infomation write
	char buf[256];
	sprintf(buf, "%s\\%s.bin", path, name);
	FILE *fp = fopen(buf, "wb");
	fwrite(&src.rows, sizeof(int), 1, fp);
	fwrite(&src.cols, sizeof(int), 1, fp);
	int Type = src.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < src.rows * src.cols; i++)		fwrite(&src.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);
}