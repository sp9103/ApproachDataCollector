#include <stdio.h>
#include <conio.h>

#include "KinectMangerThread.h"
#include "ColorBasedTracker.h"

#define DATAPATH "D:\\ApproachData"

void CreateDatadir(const char* dir);
void writeDepthData(cv::Mat src, char* path, char* name);
void writePointCloud(cv::Mat src, char* path, char* name);
void CreateApproachDir(const char* dir);

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

	int gestIdx = 0;
	while(!kinectManager.isThreadDead()){
		printf("store obj img : press 's', exit 'q'\n");
		int keyinput = getch();

		if(keyinput == (int)'s'){
			cv::Mat objImg = kinectManager.getImg();
			cv::Mat objdepth = kinectManager.getDepth();
			cv::Mat objpc = kinectManager.getPointCloud();

			//1. 저장하고
			char objpath[256], filename[256];
			sprintf(objpath, "%s\\RGB\\%d.bmp", dataDir, gestIdx);
			cv::imshow(objpath, objImg);
			imwrite(objpath, objImg);						//rgb write
			sprintf(objpath, "%s\\DEPTH", dataDir);
			itoa(gestIdx, filename, 10);
			strcat(filename, ".bin");
			writeDepthData(objdepth, objpath, filename);	//depthdata write
			sprintf(objpath, "%s\\POINTCLOUD", dataDir);
			writePointCloud(objpc, objpath, filename);

			//2. 디렉토리 생성하고
			char gestDir[256];
			sprintf(gestDir, "%s\\APPROACH\\%d", dataDir, gestIdx);
			CreateApproachDir(gestDir);	

			//3. 완료됨이 확인되면 프레임단위로 촬영
			printf("if ready to store approach img press any key\n");
			getch();
			while(1){
			}
		}
		else if(keyinput == (int)'q'){
			break;
		}

		gestIdx++;
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

void writePointCloud(cv::Mat src, char* path, char* name){
	char buf[256];
	sprintf(buf, "%s\\%s.bin", path, name);
	FILE *fp = fopen(buf, "wb");
	fwrite(&src.rows, sizeof(int), 1, fp);
	fwrite(&src.cols, sizeof(int), 1, fp);
	int Type = src.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < src.rows * src.cols; i++)
		for(int c = 0; c < src.channels(); c++)
			fwrite(&src.at<Vec3f>(i)[c], sizeof(float), 1, fp);
	fclose(fp);
}

void CreateApproachDir(const char* dir){
	//원본, depth, point cloud, process img 저장을 위한 디렉토리 생성
	TCHAR gestTDir[MAX_PATH] = {0, };
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR pointcloudDir[MAX_PATH] = {0,};
	TCHAR procImgDir[MAX_PATH] = {0,};
	char dirpath[256];
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dir, strlen(dir), gestTDir, MAX_PATH);
	CreateDirectory(gestTDir, NULL);															//create root directory
	sprintf(dirpath, "%s\\RGB", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리 - 원본
	sprintf(dirpath, "%s\\DEPTH", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리 - 원본
	sprintf(dirpath, "%s\\POINTCLOUD", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), pointcloudDir, MAX_PATH);
	CreateDirectory(pointcloudDir, NULL);											//point cloud dir - 원본
	sprintf(dirpath, "%s\\PROCIMG", dir);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), procImgDir, MAX_PATH);
	CreateDirectory(procImgDir, NULL);											//approaching point dir
}