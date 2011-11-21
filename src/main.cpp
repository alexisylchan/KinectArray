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
#include "vtkInteractionDeviceManager.h"
#include <vtkMatrix4x4.h>
#include <vtkMath.h>


#define S_WIDTH 640
#define S_HEIGHT 480
#define PIXEL_SCALE 4
#define USE_TRACKER 0
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
				if (d == 0)
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
				inputInteractor->Update(); 
		 	devMan->GetCameraDataForAllDevices(cameraDataVector);
			updatePolyData();   
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
	trackerAddress =  "tracker@localhost";//"Tracker0@tracker1-cs.cs.unc.edu";

	trackerOrigin[0] =  7.88;
	trackerOrigin[1] =  5.193477;
	trackerOrigin[2] =  1.04;
	sensorIndex = 1; 
	origSensorIndex = 1;
 
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
		camera->SetConfigParams(O2Screen,O2Right,O2Left,O2Top,O2Bottom, 0.065  ,1.0/*renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetDistance()/O2Screen*/,SurfaceRot); 
		SurfaceRot->Delete(); 
		 
}
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
		vtkVRPNTrackerCustomSensor* tracker1 = vtkVRPNTrackerCustomSensor::New();
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
   
	 devMan->GetCameraDataForAllDevices(cameraDataVector);
	updatePolyData(); 
	  
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
	iren->CreateRepeatingTimer(100);

	
	//

	//Set up camera
	//ren->ResetCamera();
	//ren->GetActiveCamera()->Roll(180.0);
	//ren->GetActiveCamera()->Azimuth(180.0);
	//ren->GetActiveCamera()->Zoom(2.0);
	    if (USE_TRACKER)
			initializeTracker();
		else
		{ 
			devMan->GetCameraDataForAllDevices(cameraDataVector);
			updatePolyData();
		}
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
