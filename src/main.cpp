#include <stdio.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkType.h>
#include <vtkCamera.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkRendererCollection.h>

#include <XnCppWrapper.h>

#include "DeviceManager.h"
#include "SensorDevice.h"

//Tracking
//#include <QTimer>
#include "vtkDeviceInteractor.h"
#include "vtkInteractionDeviceManager.h"
#include "vtkInteractorStyleTrackballCamera.h"  
#include "vtkVRPNTrackerCustomSensor.h"
#include "vtkVRPNTrackerCustomSensorStyleCamera.h"
#include "vtkVRPNTracker.h"
#include "vtkVRPNTrackerStyleCamera.h"
#include "vtkInteractionDeviceManager.h"
#include <vtkMatrix4x4.h>
#include <vtkMath.h>

#include "vtkConeSource.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h" 
#include "vtkTransform.h"

#define S_WIDTH 640
#define S_HEIGHT 480
#define PIXEL_SCALE 1
#define USE_TRACKER 1
#define RESET_CAMERA 0
#define TRACKER_UPDATE_CONE_POS 0
#define TIMER_LENGTH 4

#define DRAW_CONE 1

#define DRAW_KINECT 1


#define CULL_DEPTH 0
#define KINECT_SET_STEREO_ON 1
struct vtkRenderGroup
{
	vtkSmartPointer<vtkPolyData> polyData_;
	vtkSmartPointer<vtkPolyDataMapper> mapper_;
	vtkSmartPointer<vtkActor> actor_;
};

DeviceManager* devMan;
std::vector<CameraDataPacket> cameraDataVector;
std::vector<vtkRenderGroup> renderGroups;
vtkRenderWindow* renwin  ;
vtkRenderWindow* datawin  ;
// Headtracking
  int useTracker;
  const char* trackerAddress;
  double trackerOrigin[3];
  int sensorIndex;
  int origSensorIndex;  
  //inputInteractor 
  vtkDeviceInteractor* inputInteractor;

  //Need to reset renderer to handle load state
  vtkVRPNTrackerCustomSensorStyleCamera*  trackerStyleCamera1;
  vtkVRPNTrackerCustomSensorStyleCamera*  trackerStyleCamera2;
  vtkVRPNTrackerCustomSensor* tracker1; 
	 
	bool first = true;
	vtkActor* ConeActor;
	vtkConeSource* Cone;

	vtkMatrix4x4* kinectTransform;

/**
	Function Declaration
*/
void initializeTracker();
void initializeEyeAngle(vtkCamera* camera);
void initializeDevices();
void updatePolyData();
void timerCallback();

 
void updatePolyData()
{
	if (DRAW_CONE )
	{
	if (first)
	{
		 double position[3] = {0, 0, 0}; 
		//double position[3] = {-0.000033, -0.065609, -0.087861};
	//double quat[4] = { -0.205897 ,-0.050476, -0.227901 , 0.950326};
	    double  matrix[3][3];
		double orientNew[3] ;
		  
	/*	vtkMatrix4x4* cameraLightTransformMatrix = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix(); 
		cameraLightTransformMatrix->MultiplyPoint(position,position); */
		// Update Object Orientation

		//Change transform quaternion to matrix
		//vtkMath::QuaternionToMatrix3x3(quat, matrix); 
		//vtkMatrix4x4* RotationMatrix = vtkMatrix4x4::New();
		//RotationMatrix->SetElement(0,0, matrix[0][0]);
		//RotationMatrix->SetElement(0,1, matrix[0][1]);
		//RotationMatrix->SetElement(0,2, matrix[0][2]); 
		//RotationMatrix->SetElement(0,3, 0.0); 
		//
		//RotationMatrix->SetElement(1,0, matrix[1][0]);
		//RotationMatrix->SetElement(1,1, matrix[1][1]);
		//RotationMatrix->SetElement(1,2, matrix[1][2]); 
		//RotationMatrix->SetElement(1,3, 0.0); 
		//
		//RotationMatrix->SetElement(2,0, matrix[2][0]);
		//RotationMatrix->SetElement(2,1, matrix[2][1]);
		//RotationMatrix->SetElement(2,2, matrix[2][2]); 
		//RotationMatrix->SetElement(2,3, 1.0); 

		////cameraLightTransformMatrix->Multiply4x4(cameraLightTransformMatrix,RotationMatrix,RotationMatrix); 
		//vtkTransform::GetOrientation(orientNew,RotationMatrix); 

	// ConeSource
    Cone = vtkConeSource::New();
	Cone->SetRadius(0.5);
	Cone->SetHeight( 1.0);  
	//Cone->SetDirection(orientNew); 

	//Cone Mapper
    vtkPolyDataMapper* ConeMapper = vtkPolyDataMapper::New();
	ConeMapper->SetInput(Cone->GetOutput());
    
	ConeActor = vtkActor::New();
    ConeActor->SetMapper(ConeMapper); 
	ConeActor->SetPosition(position);   
	datawin->GetRenderers()->GetFirstRenderer()->AddActor(ConeActor);
	first = false;
	}
	else if (TRACKER_UPDATE_CONE_POS)
	{
		double* position = (double*)malloc(sizeof(double)*3);
		position = tracker1->GetPosition();
		double newPosition[3];
		//Scale up position. TODO: Determine how much to scale between phantom position and world position
		for (int s = 0; s<3;s++)
		{
			newPosition[s]=position[s];
		}

		double orientNew[3];
		double matrix[3][3];
	 
		// Update Object Orientation

		//Change transform quaternion to matrix
		vtkMath::QuaternionToMatrix3x3(tracker1->GetRotation(), matrix); 
		vtkMatrix4x4* RotationMatrix = vtkMatrix4x4::New();
		RotationMatrix->SetElement(0,0, matrix[0][0]);
		RotationMatrix->SetElement(0,1, matrix[0][1]);
		RotationMatrix->SetElement(0,2, matrix[0][2]); 
		RotationMatrix->SetElement(0,3, 0.0); 
		
		RotationMatrix->SetElement(1,0, matrix[1][0]);
		RotationMatrix->SetElement(1,1, matrix[1][1]);
		RotationMatrix->SetElement(1,2, matrix[1][2]); 
		RotationMatrix->SetElement(1,3, 0.0); 
		
		RotationMatrix->SetElement(2,0, matrix[2][0]);
		RotationMatrix->SetElement(2,1, matrix[2][1]);
		RotationMatrix->SetElement(2,2, matrix[2][2]); 
		RotationMatrix->SetElement(2,3, 1.0); 
		vtkTransform::GetOrientation(orientNew,RotationMatrix);
		ConeActor->SetOrientation(orientNew); 

		double normalizedOrientNew= vtkMath::Norm(orientNew);
		for (int i =0; i < 3; i++)
		{
			newPosition[i] = newPosition[i] -(Cone->GetHeight()/2.0)* (orientNew[i]/normalizedOrientNew);
		}
		ConeActor->SetPosition(newPosition);
		double* cameraposition ;
		cameraposition = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition( );
		 
	}
	else
	{
		ConeActor->Modified();
		bool headtracked ;
		headtracked = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetHeadTracked();
		vtkMatrix4x4* viewMatrix ;
		viewMatrix = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetViewTransformMatrix();
	}
	}
	if (DRAW_KINECT)
	{
	for(unsigned int i = 0; i < cameraDataVector.size(); i++)
	{
	
		// need to create a new render group for any new camera device
		if (renderGroups.size() <= i)
		{
			vtkRenderGroup renderGroup;
			renderGroup.polyData_ = vtkSmartPointer<vtkPolyData>::New();
			renderGroup.mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
			renderGroup.actor_ = vtkSmartPointer<vtkActor>::New();
			
			renderGroup.mapper_->SetInput(renderGroup.polyData_);
			renderGroup.actor_->SetMapper(renderGroup.mapper_);
			renwin->GetRenderers()->GetFirstRenderer()->AddActor(renderGroup.actor_);
			
			renderGroups.push_back(renderGroup);
		}
		double* cameraposition ;
		cameraposition = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition( );
		
		vtkMatrix4x4* viewMatrix ;
		viewMatrix = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix();

		double* orientation;
		orientation = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetOrientation();

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkUnsignedCharArray> color = vtkSmartPointer<vtkUnsignedCharArray>::New();
		color->SetName("Color");
		color->SetNumberOfComponents(3);
	
		
		// These coordinates are in model view!!!!
		for(int y = 0; y < S_HEIGHT; y++)
		{
			for(int x = 0; x < S_WIDTH; x++)
			{

				if (!RESET_CAMERA)
				{
					XnDepthPixel d = cameraDataVector[i].depthMap_[x+y*S_WIDTH];
					XnRGB24Pixel c = cameraDataVector[i].imageMap_[x+y*S_WIDTH];
		
					vtkIdType id;

					double pixPos[3];
					pixPos[0] = ((double)x)/100.0; pixPos[1] = ((double)y)/100.0; pixPos[2] = d*0.001-1.0;
					
					if (d == 0  || (CULL_DEPTH && d > 2000))/*(d == 0)*/
					{
						id = points->InsertNextPoint(pixPos[0],pixPos[1],-1.0);
						color->InsertNextTuple3(0,0,0);
					}
					else  
					{
						id = points->InsertNextPoint(pixPos[0],pixPos[1],pixPos[2]);
						color->InsertNextTuple3(c.nRed, c.nGreen, c.nBlue);
					}
		
					cellArray->InsertNextCell(1);
					cellArray->InsertCellPoint(id);
				}
				else
				{
					XnDepthPixel d = cameraDataVector[i].depthMap_[x+y*S_WIDTH];
					XnRGB24Pixel c = cameraDataVector[i].imageMap_[x+y*S_WIDTH];
		
					vtkIdType id;
					if (d == 0  || (CULL_DEPTH && d > 2000))/*(d == 0)*/
					{
						id = points->InsertNextPoint(x,y,4096*.3);
						color->InsertNextTuple3(0,0,0);
					}
					else  
					{
						id = points->InsertNextPoint(x*PIXEL_SCALE,y*PIXEL_SCALE,/*d*0.001*/d*.3);
						color->InsertNextTuple3(c.nRed, c.nGreen, c.nBlue);
					}
		
					cellArray->InsertNextCell(1);
					cellArray->InsertCellPoint(id);
				}
			}
		}
	
		renderGroups[i].polyData_->SetPoints(points);
		renderGroups[i].polyData_->SetVerts(cellArray);
		renderGroups[i].polyData_->GetPointData()->SetScalars(color);
		renderGroups[i].polyData_->Modified();
		renderGroups[i].polyData_->Update();
	}
}
}

class UpdateData:public vtkCommand
{
	public:
		static UpdateData *New()
		{
			UpdateData* up = new UpdateData;
			return up;
		}
		virtual void Execute(vtkObject* caller, unsigned long eid, void* clientdata)
		{ 
		    if (USE_TRACKER)
			{
				inputInteractor->Update(); 
			}
		 	devMan->GetCameraDataForAllDevices(cameraDataVector);
			updatePolyData();   
			renwin->Render();
			datawin->Render();
		}
};

void timerCallback()
{
	devMan->GetCameraDataForAllDevices(cameraDataVector);
	updatePolyData(); 
	inputInteractor->Update(); 
}
void initializeTracker()
{
	//Tracker Options
	useTracker = 1;

	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/
	trackerAddress =  "tracker@localhost";//"Tracker0@tracker1-cs.cs.unc.edu";
	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/

	trackerOrigin[0] =  -7.51;
	trackerOrigin[1] =  -5.16;
	trackerOrigin[2] =  -0.99;
	sensorIndex = 0; 
	origSensorIndex = 0;
 
	initializeEyeAngle( renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
	initializeEyeAngle( datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
	initializeDevices( ); 
}
void initializeEyeAngle(vtkCamera* camera)
{      
	
		camera->SetEyeAngle(0);
		camera->SetHeadTracked(true);//useTracker);

		//Code from vtkCaveSynchronizedRenderers::SetDisplayConfig
		double DisplayX[4], DisplayY[4],DisplayOrigin[4];

		 
			
			DisplayOrigin[0]= 8.174900;
		 	DisplayOrigin[1]= 5.435796; 
			DisplayOrigin[2]= 0.808637;//-value; 

			DisplayX[0]=   8.192622;//value; 
			DisplayX[1]=    4.967024;//-value; 
			DisplayX[2]=    0.803074;//-value; 

			DisplayY[0]= 8.197782;//value; 
			DisplayY[1]=    4.963459;//value; 
			DisplayY[2]=     1.094415;//-value;     

			//Todo: factorize this
			vtkMatrix4x4* trackerTransformM = vtkMatrix4x4::New();
			trackerTransformM->SetElement(0,0,0);
			trackerTransformM->SetElement(0,1,-1);
			trackerTransformM->SetElement(0,2,0);
			trackerTransformM->SetElement(0,3,-1*trackerOrigin[1]);
			trackerTransformM->SetElement(1,0,0);
			trackerTransformM->SetElement(1,1,0);
			trackerTransformM->SetElement(1,2,1);
			trackerTransformM->SetElement(1,3, 1*trackerOrigin[2]);
			trackerTransformM->SetElement(2,0,-1);
			trackerTransformM->SetElement(2,1,0);
			trackerTransformM->SetElement(2,2,0); 
			trackerTransformM->SetElement(2,3,-1*trackerOrigin[0]);
			trackerTransformM->SetElement(3,0, 0);
			trackerTransformM->SetElement(3,1, 0 );
			trackerTransformM->SetElement(3,2,0); 
			trackerTransformM->SetElement(3,3,1);
			DisplayOrigin[3] = 1;
			DisplayX[3] = 1;
			DisplayY[3] = 1;
			trackerTransformM->MultiplyPoint(DisplayOrigin,DisplayOrigin);
			trackerTransformM->MultiplyPoint(DisplayX,DisplayX);
			trackerTransformM->MultiplyPoint(DisplayY,DisplayY); 
		 

		double xBase[3],yBase[3],zBase[3];
		//Get Vectors of screen
		for (int i =0; i < 3; ++i)
		{
		xBase[i] = DisplayX[i]-DisplayOrigin[i];
		yBase[i] = DisplayY[i]-DisplayX[i];
		}
		vtkMath::Cross(xBase,yBase,zBase);
		//Code from vtkCaveSynchronizedRenderers::SetSurfaceRotation
		vtkMatrix4x4* SurfaceRot = vtkMatrix4x4::New();
		vtkMath::Normalize( xBase );
		vtkMath::Normalize( yBase );
		vtkMath::Normalize( zBase );

		SurfaceRot->SetElement( 0, 0, xBase[0] );
		SurfaceRot->SetElement( 0, 1, xBase[1] );
		SurfaceRot->SetElement( 0, 2, xBase[2] );

		SurfaceRot->SetElement( 1, 0, yBase[0] );
		SurfaceRot->SetElement( 1, 1, yBase[1] );
		SurfaceRot->SetElement( 1, 2, yBase[2] );

		SurfaceRot->SetElement( 2, 0, zBase[0]);
		SurfaceRot->SetElement( 2, 1, zBase[1]);
		SurfaceRot->SetElement( 2, 2, zBase[2]);
		SurfaceRot->MultiplyPoint( DisplayOrigin, DisplayOrigin );
		SurfaceRot->MultiplyPoint( DisplayX, DisplayX );
		SurfaceRot->MultiplyPoint( DisplayY, DisplayY );  
		// Set O2Screen, O2Right, O2Left, O2Bottom, O2Top
		double O2Screen = - DisplayOrigin[2];
		double O2Right  =   DisplayX[0];
		double O2Left   = - DisplayOrigin[0];
		double O2Top    =   DisplayY[1];
		double O2Bottom = - DisplayX[1]; 
		camera->SetConfigParams(O2Screen,O2Right,O2Left,O2Top,O2Bottom, 0.065  ,6.69/O2Screen/*renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetDistance()/O2Screen*/,SurfaceRot); 
		SurfaceRot->Delete(); 
		 
} 
void initializeDevices()
{ 

	/////////////////////////INTERACTOR////////////////////////////
	// Initialize Device Interactor to manage all trackers
    inputInteractor = vtkDeviceInteractor::New();
		/////////////////////////CREATE  TRACKER////////////////////////////

		//Create connection to VRPN Tracker using vtkInteractionDevice.lib
		tracker1 = vtkVRPNTrackerCustomSensor::New();
		tracker1->SetDeviceName(trackerAddress); 
		tracker1->SetSensorIndex(sensorIndex);//TODO: Fix error handling  
		tracker1->SetTracker2WorldTranslation(trackerOrigin[0],trackerOrigin[1],trackerOrigin[2]);
		double t2w1[3][3] = { 0, -1,  0,
							  0,  0, 1, 
							 -1, 0,  0 }; 
		double t2wQuat1[4];
		vtkMath::Matrix3x3ToQuaternion(t2w1, t2wQuat1);
		tracker1->SetTracker2WorldRotation(t2wQuat1);
		

		int trackerInitResult = tracker1->Initialize();
		if (trackerInitResult == 0)
		{
			printf("Error initializing tracker ");			 
		}

		/////////////////////////CREATE  TRACKER STYLE////////////////////////////

		//Create device interactor style (defined in vtkInteractionDevice.lib) that determines how the device manipulates camera viewpoint
		trackerStyleCamera1 = vtkVRPNTrackerCustomSensorStyleCamera::New();
		trackerStyleCamera1->SetTracker(tracker1);
		trackerStyleCamera1->SetRenderer(renwin->GetRenderers()->GetFirstRenderer());
			//Create device interactor style (defined in vtkInteractionDevice.lib) that determines how the device manipulates camera viewpoint
		trackerStyleCamera2 = vtkVRPNTrackerCustomSensorStyleCamera::New();
		trackerStyleCamera2->SetTracker(tracker1);
		trackerStyleCamera2->SetRenderer(datawin->GetRenderers()->GetFirstRenderer());

	
		/////////////////////////INTERACTOR////////////////////////////
		//Register Tracker to Device Interactor
		inputInteractor->AddInteractionDevice(tracker1);
		inputInteractor->AddDeviceInteractorStyle(trackerStyleCamera1);  
		inputInteractor->AddDeviceInteractorStyle(trackerStyleCamera2);
}

int main(int argc, char** argv)
{
	
	devMan = new DeviceManager();
	
	unsigned int i = devMan->GetNODevicesConnected();
	
	if (i < 1)
	{
		std::cout << "No devices found." << std::endl;
		return -1;
	}
	
	printf("Number of devices connected: %d.\n", i);
	//VTK Pipeline
	vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderer> dataren = vtkSmartPointer<vtkRenderer>::New();
	renwin = vtkRenderWindow::New();
	datawin= vtkRenderWindow::New();
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> data_iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	
	if (KINECT_SET_STEREO_ON)
	{
		renwin->SetStereoCapableWindow(1);
		renwin->StereoRenderOn();
	}
	renwin->AddRenderer(ren);
	renwin->SetInteractor(iren);
	renwin->SetSize(1280,800);
	iren->Initialize();
	datawin->AddRenderer(dataren);
	datawin->SetInteractor(data_iren);
	datawin->SetSize(1280,800);
	data_iren->Initialize();


	//Add timer callback
	vtkSmartPointer<UpdateData> updateCallback = vtkSmartPointer<UpdateData>::New();
	iren->AddObserver(vtkCommand::TimerEvent, updateCallback);
	iren->CreateRepeatingTimer(TIMER_LENGTH); 
	
	devMan->GetCameraDataForAllDevices(cameraDataVector); 
	updatePolyData(); 

	if (RESET_CAMERA)
	{  
	ren->ResetCamera();
	ren->GetActiveCamera()->Roll(180.0);
	ren->GetActiveCamera()->Azimuth(180.0);
	ren->GetActiveCamera()->Zoom(2.0); 
	}
	else
	{
	ren->GetActiveCamera()->Roll(180.0);
	ren->GetActiveCamera()->Azimuth(180.0);
	ren->GetActiveCamera()->SetPosition(0,0,6.69);
	ren->GetActiveCamera()->Modified();

	
	dataren->GetActiveCamera()->Roll(180.0);
	dataren->GetActiveCamera()->Azimuth(180.0);
	dataren->GetActiveCamera()->SetPosition(0,0,6.69);
	dataren->GetActiveCamera()->Modified();
	} 
	 
	
	if (USE_TRACKER)
			initializeTracker();   
	/*iren->Start();
	data_iren->Start();*/
 
//	  // Start interacting
  MSG msg;
    while (1) {
        PeekMessage(&msg, NULL, 0, 0, PM_REMOVE);
        TranslateMessage(&msg);
        DispatchMessage(&msg);

       
    }
	delete devMan;
	
	return 0;
}
