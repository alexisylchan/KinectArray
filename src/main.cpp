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

#define CULL_DEPTH 0
struct vtkRenderGroup
{
	vtkSmartPointer<vtkPolyData> polyData_;
	vtkSmartPointer<vtkPolyDataMapper> mapper_;
	vtkSmartPointer<vtkActor> actor_;
};

DeviceManager* devMan;
std::vector<CameraDataPacket> cameraDataVector;
std::vector<vtkRenderGroup> renderGroups;
vtkSmartPointer<vtkRenderWindow> renwin;

//vtkRenderWindow*  renwin;
//vtkRenderer* renderer1;



// Headtracking
  int useTracker;
  const char* trackerAddress;
  double trackerOrigin[3];
  int sensorIndex;
  int origSensorIndex; 
  //QTimer *VRPNTimer;
  //inputInteractor 
  vtkDeviceInteractor* inputInteractor;

  //Need to reset renderer to handle load state
  vtkVRPNTrackerCustomSensorStyleCamera*  trackerStyleCamera1;
  vtkVRPNTrackerCustomSensor* tracker1;
  //Need to reset renderer to handle load state
  //vtkVRPNTrackerStyleCamera*  trackerStyleCamera1;
  //vtkVRPNTracker* tracker1;
	 
bool first = true;
vtkActor* ConeActor;
vtkConeSource* Cone;
/**
	Function Declaration
*/
void initializeTracker();
void initializeEyeAngle();
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
	renwin->GetRenderers()->GetFirstRenderer()->AddActor(ConeActor);
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
	/*	vtkMatrix4x4* cameraLightTransformMatrix = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix(); 
		cameraLightTransformMatrix->MultiplyPoint(newPosition,newPosition); */
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
		cameraposition = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition( );
		 
	}
	else
	{
		ConeActor->Modified();
		bool headtracked ;
		headtracked = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetHeadTracked();
		vtkMatrix4x4* viewMatrix ;
		viewMatrix = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetViewTransformMatrix();
	}
	}
	else
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
		
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkUnsignedCharArray> color = vtkSmartPointer<vtkUnsignedCharArray>::New();
		color->SetName("Color");
		color->SetNumberOfComponents(3);
	
		for(int y = 0; y < S_HEIGHT; y++)
		{
			for(int x = 0; x < S_WIDTH; x++)
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
					id = points->InsertNextPoint(x*PIXEL_SCALE,y*PIXEL_SCALE,d*.3);
					color->InsertNextTuple3(c.nRed, c.nGreen, c.nBlue);
				}
	
				cellArray->InsertNextCell(1);
				cellArray->InsertCellPoint(id);
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
			//polyData->Initialize();
			//devMan->GetCameraDataByDeviceIndex(0, &dataPacket);
			//inputInteractor->Update();
		    if (USE_TRACKER)
			{
				inputInteractor->Update(); 
			}
		 	devMan->GetCameraDataForAllDevices(cameraDataVector);
			updatePolyData();  
			//inputInteractor->Update(); 
			renwin->Render();
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
 
	initializeEyeAngle();
	initializeDevices();
}
void initializeEyeAngle()
{      
	vtkCamera* camera = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
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
//void initializeDevices()
//{ 
//   
//	  
//
//	/////////////////////////INTERACTOR////////////////////////////
//	// Initialize Device Interactor to manage all trackers
//    inputInteractor = vtkDeviceInteractor::New();
//		/////////////////////////CREATE  TRACKER////////////////////////////
//
//		//Create connection to VRPN Tracker using vtkInteractionDevice.lib
//		tracker1 = vtkVRPNTracker::New();
//		tracker1->SetDeviceName(trackerAddress); 
//		//tracker1->SetSensorIndex(sensorIndex);//TODO: Fix error handling  
//		tracker1->SetTracker2WorldTranslation(trackerOrigin[0],trackerOrigin[1],trackerOrigin[2]);
//		double t2w1[3][3] = { 0, -1,  0,
//							  0,  0, 1, 
//							 -1, 0,  0 }; 
//		double t2wQuat1[4];
//		vtkMath::Matrix3x3ToQuaternion(t2w1, t2wQuat1);
//		tracker1->SetTracker2WorldRotation(t2wQuat1);
//		
//
//		int trackerInitResult = tracker1->Initialize();
//		if (trackerInitResult == 0)
//		{
//			printf("Error initializing tracker ");			 
//		}
//
//		/////////////////////////CREATE  TRACKER STYLE////////////////////////////
//
//		//Create device interactor style (defined in vtkInteractionDevice.lib) that determines how the device manipulates camera viewpoint
//		trackerStyleCamera1 = vtkVRPNTrackerStyleCamera::New();
//		trackerStyleCamera1->SetTracker(tracker1);
//		trackerStyleCamera1->SetRenderer(renwin->GetRenderers()->GetFirstRenderer());
//	
//		/////////////////////////INTERACTOR////////////////////////////
//		//Register Tracker to Device Interactor
//		inputInteractor->AddInteractionDevice(tracker1);
//		inputInteractor->AddDeviceInteractorStyle(trackerStyleCamera1); 
//   
//	/* devMan->GetCameraDataForAllDevices(cameraDataVector);
//	updatePolyData(); */
//	  
//   /* connect(VRPNTimer,SIGNAL(timeout()),
//		 this,SLOT(timerCallback()));
//    VRPNTimer->start();*/
//	
//    
////} 
//}

void initializeDevices()
{
// VRPN input events.
	//VRPNTimer=new QTimer(this);
	//VRPNTimer->setInterval(4); // in ms
   
	  

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
	
		/////////////////////////INTERACTOR////////////////////////////
		//Register Tracker to Device Interactor
		inputInteractor->AddInteractionDevice(tracker1);
		inputInteractor->AddDeviceInteractorStyle(trackerStyleCamera1); 
   
	/* devMan->GetCameraDataForAllDevices(cameraDataVector);
	updatePolyData(); */
	  
   /* connect(VRPNTimer,SIGNAL(timeout()),
		 this,SLOT(timerCallback()));
    VRPNTimer->start();*/
	
    
//} 
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
	renwin = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renwin->AddRenderer(ren);
	renwin->SetInteractor(iren);
	renwin->SetSize(1280,800);
	iren->Initialize();

	//Add timer callback
	vtkSmartPointer<UpdateData> updateCallback = vtkSmartPointer<UpdateData>::New();
	iren->AddObserver(vtkCommand::TimerEvent, updateCallback);
	iren->CreateRepeatingTimer(TIMER_LENGTH);

	
	//

	
	devMan->GetCameraDataForAllDevices(cameraDataVector);
	updatePolyData();
	/*if (DRAW_CONE)
	{*/
		//Set up camera

	if (RESET_CAMERA)
	{
	ren->ResetCamera();
	ren->GetActiveCamera()->Roll(180.0);
	ren->GetActiveCamera()->Azimuth(180.0);
	ren->GetActiveCamera()->Zoom(2.0);
	}
	double* position = ren->GetActiveCamera()->GetPosition();
	/*}*/
	
	ren->GetActiveCamera()->SetPosition(0,0,6.69);
	ren->GetActiveCamera()->Modified();
	if (USE_TRACKER)
			initializeTracker(); 
	 
	   /*if (USE_TRACKER)
			initializeTracker();
		else
		{ 
			devMan->GetCameraDataForAllDevices(cameraDataVector);
			updatePolyData();
		}*/
	iren->Start();



//	//VTK Pipeline
//	renderer1 = vtkRenderer::New();
//	renwin = vtkRenderWindow::New();
//	
//	renwin->AddRenderer(renderer1);
//	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::New();
//	renwin->SetInteractor(iren);
//	renwin->SetSize(1280,800);
//
//	iren->Initialize();
//
//	/*vtkInteractionDeviceManager* manager = vtkInteractionDeviceManager::New();
//
//   vtkRenderWindowInteractor* iren = manager->GetInteractor(inputInteractor);*/
//	
//	/*
//	renwin->SetInteractor(iren);
//	vtkInteractorStyleTrackballCamera* interactorStyle = vtkInteractorStyleTrackballCamera::New();
//*/
//	//iren->SetInteractorStyle(interactorStyle);
//
//
//	//vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//
//
//	/*iren->Initialize();*/
//
//	////Add timer callback
//	vtkSmartPointer<UpdateData> updateCallback = vtkSmartPointer<UpdateData>::New();
//	iren->AddObserver(vtkCommand::TimerEvent, updateCallback);
//	iren->CreateRepeatingTimer(100);
//
//	
//	//
//	////devMan->GetCameraDataByDeviceIndex(0, &dataPacket);
//	devMan->GetCameraDataForAllDevices(cameraDataVector);
//	updatePolyData();
//	////Set up camera
//	renderer1->ResetCamera();
//	renderer1->GetActiveCamera()->Roll(180.0);
//	renderer1->GetActiveCamera()->Azimuth(180.0);
//	renderer1->GetActiveCamera()->Zoom(2.0);
//
//	    if (USE_TRACKER)
//			initializeTracker();
//	
// //
//	iren->Start();
//	
	delete devMan;
	
	return 0;
}
