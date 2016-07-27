#include <stdio.h>
#include <conio.h>

#include "KinectMangerThread.h"
#include "ColorBasedTracker.h"

#define DATAPATH "D:\\ApproachData"

void CreateDatadir(const char* dir);
void writeDepthData(cv::Mat src, char* path, char* name);
void writePointCloud(cv::Mat src, char* path, char* name);
void CreateApproachDir(const char* dir);
void writeFrameData(const char* dir, cv::Mat RGB, cv::Mat depth, cv::Mat pc, cv::Mat procImg, int count);
void removeObj(cv::Mat backRGB, cv::Mat BackDepth,
			   cv::Mat objRGB, cv::Mat objDepth,
			   cv::Mat frame, cv::Mat frameDepth,
			   cv::Mat *dstColor, cv::Mat *dstDepth);

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
	//1. background 저장하고
	char backPath[256];
	sprintf(backPath, "%s\\background.bmp", dataDir);
	rgbBack = kinectManager.getImg();
	depthBack = kinectManager.getDepth();
	pcBack = kinectManager.getPointCloud();
	tracker.InsertBackGround(rgbBack, depthBack);		//tracker initialize
	imwrite(backPath, rgbBack);							//rgb 배경저장
	writeDepthData(depthBack, dataDir, "background");
	printf("%d chan\n", rgbBack.channels());

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
			cv::waitKey(1);
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

			int count = 0;
			while(1){
				cv::Mat frameImg = kinectManager.getImg();
				cv::Mat frameDepth = kinectManager.getDepth();
				cv::Mat framepc = kinectManager.getPointCloud();
				cv::Mat objRmvRGB, objRmvDepth;
				removeObj(rgbBack, depthBack, frameImg, frameDepth, objImg, objdepth, &objRmvRGB, &objRmvDepth);
				cv::imshow("objRmv", objRmvRGB);
				cv::Mat frameprocImg = tracker.calcImage(objRmvRGB, objRmvDepth);

				cv::imshow("proc", frameImg);
				char cvKey = cv::waitKey(50);
				if(cvKey == 'q')
					break;
				else if(cvKey == 's'){
					cv::waitKey(0);
				}

				if(frameprocImg.rows == 0)	continue;

				printf("[%d] data save.\n", count);
				writeFrameData(gestDir, frameImg, frameDepth, framepc, frameprocImg, count++);
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

void writeFrameData(const char* dir, cv::Mat RGB, cv::Mat depth, cv::Mat pc, cv::Mat procImg, int count){
	//input : gesture dir 
	char buf[256], countBuf[256];
	itoa(count, countBuf, 10);
	sprintf(buf, "%s\\RGB\\%d.bmp", dir, count);
	cv::imwrite(buf, RGB);
	sprintf(buf, "%s\\PROCIMG\\%d.bmp", dir, count);
	cv::imwrite(buf, procImg);
	sprintf(buf, "%s\\POINTCLOUD", dir);
	writePointCloud(pc, buf, countBuf);
	sprintf(buf, "%s\\DEPTH", dir);
	writeDepthData(depth, buf, countBuf);
}

void removeObj(cv::Mat backRGB, cv::Mat BackDepth, cv::Mat frame, cv::Mat frameDepth, cv::Mat objRGB, cv::Mat objDepth, cv::Mat *dstColor, cv::Mat *dstDepth){
	const int rows = frame.rows;
	const int cols = frame.cols;
	const float threshold = 5;
	const int cThreshold = 40;

	dstColor->create(frame.rows, frame.cols, frame.type());
	dstDepth->create(frameDepth.rows, frameDepth.cols, frameDepth.type());

	for(int h = 0; h < rows; h++){
		for(int w = 0; w < cols; w++){
			cv::Vec4b backRGBval = backRGB.at<cv::Vec4b>(h,w);
			cv::Vec4b objRGBval = objRGB.at<cv::Vec4b>(h,w);
			cv::Vec4b frameval = frame.at<cv::Vec4b>(h,w);

			cv::Vec4b objSubRGB = cv::Vec4b(abs(backRGBval[0] - objRGBval[0]), abs(backRGBval[1] - objRGBval[1]), abs(backRGBval[2] - objRGBval[2]), abs(backRGBval[3] - objRGBval[3]));

			float backDepthval = BackDepth.at<float>(h,w);
			float objDepthval = objDepth.at<float>(h,w);
			float frameDepthval = frameDepth.at<float>(h,w);

			float objSubDepth = abs(backDepthval - objDepthval);

			//Depth 먼저 판별
			if (backDepthval == 0 || objDepthval == 0){
				dstColor->at<cv::Vec4b>(h, w) = frameval;
				dstDepth->at<float>(h,w) = frameDepthval;
				continue;
			}
			if (objSubDepth > threshold){
				//color 비교
				if (objSubRGB[0] < cThreshold && objSubRGB[1] < cThreshold && objSubRGB[2] < cThreshold){
					dstColor->at<cv::Vec4b>(h, w) = frameval;
					dstDepth->at<float>(h,w) = frameDepthval;
				}
				else{
					dstColor->at<cv::Vec4b>(h, w) = backRGBval;
					dstDepth->at<float>(h,w) = backDepthval;
				}
			}else{
				dstColor->at<cv::Vec4b>(h, w) = frameval;
				dstDepth->at<float>(h,w) = frameDepthval;
			}
		}
	}
}