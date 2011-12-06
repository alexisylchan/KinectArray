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

#include "vtkCubeSource.h" 

#include "vtkPolyDataMapper.h"
#include "vtkActor.h" 
#include "vtkTransform.h"

#include "vtkProperty.h"

#include "vtkLight.h"
#include "vtkLightKit.h"

#include <vrpn_Analog.h> 

#include "vtkVRPNPhantom.h"
#include "vtkVRPNPhantomStyleCamera.h"

#define S_WIDTH 640
#define S_HEIGHT 480
#define PIXEL_SCALE 2
#define USE_TRACKER 1
#define USE_HEAD_TRACKER 0
#define RESET_CAMERA 1
#define TRACKER_UPDATE_CONE_POS 0
#define TIMER_LENGTH 1

#define DRAW_CUBE 1

#define DRAW_KINECT 0 

#define CULL_DEPTH 0
#define KINECT_SET_STEREO_ON 0
#define USE_TNG 0
#define USE_PHANTOM 1


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

  vtkVRPNPhantomStyleCamera* phantomStyleCamera1;
	 
	bool first = true;
	vtkActor* ConeActor;
	vtkConeSource* Cone;
	vtkActor* CubeActor;
	vtkCubeSource* CubeSource;

	vtkMatrix4x4* kinectTransform;

	vtkLight* MainLight;
	vtkLightKit* LightKit;

	vrpn_Analog_Remote* tng1;
	
	class tng_user_callback
	{
	public:
	  char tng_name[vrpn_MAX_TEXT_LEN];
	  vtkstd::vector<unsigned> tng_counts;
	  int channelIndex;
	  double initialValue;
	};

	tng_user_callback *TNGC1;
	void VRPN_CALLBACK handleTNG(void *userdata,
	const vrpn_ANALOGCB t)
	{
	  tng_user_callback *tData=static_cast<tng_user_callback *>(userdata); 

	  // TNG 3B values go from 0 to 252. To get an eye separation of 0.0065 to be in the middle (126), we use delta  == 0.0065/126 ==
		double value = t.channel[tData->channelIndex];  
		double delta = value; /* - tData->initialValue; */
		vtkCamera* camera = renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		camera->SetEyeOffset( (((camera->GetDistance()/camera->O2Screen)*0.065/2.0)/126.0)*delta);//Initial Camera Offset is 0, so no need to add to the initial camera offset
				
		camera = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		camera->SetEyeOffset( (((camera->GetDistance()/camera->O2Screen)*0.065/2.0)/126.0)*delta);//Initial Camera Offset is 0, so no need to add to the initial camera offset
				

	}
	

/**
	Function Declaration
*/
void initializeLights();

void initializeTracker();
void initializeTNG();
void initializeEyeAngle(vtkCamera* camera);
void initializeDevices();

void updatePolyData();
void timerCallback();

  
void createCone(bool deleteOldCone)
{
	
	double position[3] = {-0.000033, -0.065609, -0.087861};
	double quat[4] = { -0.205897 ,-0.050476, -0.227901 , 0.950326};
	    double  matrix[3][3];
		double orientNew[3] ;
		if (deleteOldCone)
		{
			ConeActor->Delete();
			Cone->Delete();
		}
		 
		vtkMatrix4x4* cameraLightTransformMatrix = datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetCameraLightTransformMatrix(); 
		cameraLightTransformMatrix->MultiplyPoint(position,position); 
		// Update Object Orientation

		//Change transform quaternion to matrix
		vtkMath::QuaternionToMatrix3x3(quat, matrix); 
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

		//cameraLightTransformMatrix->Multiply4x4(cameraLightTransformMatrix,RotationMatrix,RotationMatrix); 
		vtkTransform::GetOrientation(orientNew,RotationMatrix); 
 
	// ConeSource
    Cone = vtkConeSource::New(); 
	Cone->SetRadius(0.05);
	Cone->SetHeight(0.1);  
	Cone->SetDirection(orientNew); 

	//Cone Mapper
    vtkPolyDataMapper* ConeMapper = vtkPolyDataMapper::New();
	ConeMapper->SetInput(Cone->GetOutput());
    
	ConeActor = vtkActor::New();
    ConeActor->SetMapper(ConeMapper); 
	ConeActor->SetPosition(position); 
	ConeActor->UseBoundsOff();
	ConeActor->Modified();
 	vtkRenderer* renderer1 = datawin->GetRenderers()->GetFirstRenderer();
	renderer1->AddActor(ConeActor);

	Cone->Delete(); 
	ConeMapper->Delete();


}


void updatePolyData()
{
	/*if (USE_PHANTOM)
	{ 
		createCone(false);
	}*/
	if (DRAW_CUBE )
	{
		if (first)
		{
			 double position[3] = {0, 0, 0};  
			CubeSource = vtkCubeSource::New(); 
			//Cone->SetDirection(orientNew); 

			//Cone Mapper
			vtkPolyDataMapper* CubeMapper = vtkPolyDataMapper::New();
			CubeMapper->SetInput(CubeSource->GetOutput());
		    
			CubeActor = vtkActor::New();
			CubeActor->SetMapper(CubeMapper); 
			CubeActor->SetPosition(position); 
			CubeActor->GetProperty()->SetOpacity(0.5);
	  
			datawin->GetRenderers()->GetFirstRenderer()->AddActor(CubeActor); 
			datawin->GetRenderers()->GetFirstRenderer()->ResetCamera();
			first = false;
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
			if (USE_TNG)
			{
				tng1->mainloop();
			}
		    if (USE_TRACKER)
			{
				inputInteractor->Update(); 
			}
			
			if (DRAW_KINECT)
			{
		 		devMan->GetCameraDataForAllDevices(cameraDataVector);
			}
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

void initializeLights()
{ 
  LightKit = vtkLightKit::New();
  LightKit->AddLightsToRenderer(datawin->GetRenderers()->GetFirstRenderer());
}
void initializeTracker()
{
	//Tracker Options
	useTracker = 1;

	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/
	//trackerAddress =  "Tracker0@tracker1-cs.cs.unc.edu";
	trackerAddress =  "tracker@1";//
	/********************** CHANGE THE SENSOR INDEX WHEN CHANGING THE HOST ********************/

	trackerOrigin[0] =  -7.88;
	trackerOrigin[1] =  -5.19;
	trackerOrigin[2] =  -1.04;
		
	sensorIndex = 0; 
	origSensorIndex = 0;
 
	initializeEyeAngle( renwin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
	initializeEyeAngle( datawin->GetRenderers()->GetFirstRenderer()->GetActiveCamera() );
	initializeDevices( ); 
}

void initializeTNG()
{
		//TNG  
		char tngAddress[] = "tng3name@localhost";
		tng1 = new vrpn_Analog_Remote(tngAddress);
		TNGC1 = new tng_user_callback;
		TNGC1->channelIndex = sensorIndex; //TODO: Should this be origSensorIndex? How often do we reinitialize device 
		TNGC1->initialValue = 0;
		strncpy(TNGC1->tng_name,tngAddress,sizeof(TNGC1->tng_name));
		tng1->register_change_handler(TNGC1,handleTNG);

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
		camera->SetConfigParams(O2Screen,O2Right,O2Left,O2Top,O2Bottom, 0.065  ,/*6.69*/(camera->GetDistance()/(2*O2Screen)),SurfaceRot);
		camera->Modified(); 
		SurfaceRot->Delete();
		 
} 
void initializeDevices()
{ 

	/////////////////////////INTERACTOR////////////////////////////
	// Initialize Device Interactor to manage all trackers
    inputInteractor = vtkDeviceInteractor::New();
		/////////////////////////CREATE  TRACKER////////////////////////////

		if (USE_HEAD_TRACKER)
		{
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
		 
		if (USE_PHANTOM)
		{
		vtkVRPNPhantom* phantom1 = vtkVRPNPhantom::New();
		phantom1->SetDeviceName("Phantom0@localhost");
		phantom1->SetPhantom2WorldTranslation(0.000264,0.065412,0.0);//TODO: FIX
		phantom1->SetNumberOfButtons(2);
		phantom1->SetSensorIndex(sensorIndex);

		phantom1->Initialize();
		createCone(false);

		/////////////////////////CREATE  PHANTOM STYLE////////////////////////////
		phantomStyleCamera1 = vtkVRPNPhantomStyleCamera::New(); 
		/////////////////////////CONNECT TO SERVER CHANGE////////////////////////////
 

		phantomStyleCamera1->SetActor(ConeActor); 
		phantomStyleCamera1->SetConeSource(Cone);
 
		phantomStyleCamera1->SetPhantom(phantom1);
		phantomStyleCamera1->SetRenderer(datawin->GetRenderers()->GetFirstRenderer()); 

		
	    /////////////////////////INTERACTOR////////////////////////////
		//Register Phantom to Device Interactor 
		inputInteractor->AddInteractionDevice(phantom1);
		inputInteractor->AddDeviceInteractorStyle(phantomStyleCamera1); 
		}
}

int main(int argc, char** argv)
{
	
	if (DRAW_KINECT)
	{
	devMan = new DeviceManager();
	
	unsigned int i = devMan->GetNODevicesConnected();
	
	if (i < 1)
	{
		std::cout << "No devices found." << std::endl;
		return -1;
	}
	
	printf("Number of devices connected: %d.\n", i);
	}
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
		
		datawin->SetStereoCapableWindow(1);
		datawin->StereoRenderOn();
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
	
	initializeLights();
	if (DRAW_KINECT)
	{
		devMan->GetCameraDataForAllDevices(cameraDataVector); 
	}
	updatePolyData(); 

	if (RESET_CAMERA)
	{  
	ren->ResetCamera();
	ren->GetActiveCamera()->Roll(180.0);
	ren->GetActiveCamera()->Azimuth(180.0);
	ren->GetActiveCamera()->Zoom(2.0); 
	dataren->ResetCamera();
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
	 
	   
	if (USE_TNG)
		initializeTNG();
	if (USE_TRACKER)
			initializeTracker(); 
	// Continue  interacting
  MSG msg;
    while (1) {
        PeekMessage(&msg, NULL, 0, 0, PM_REMOVE);
        TranslateMessage(&msg);
        DispatchMessage(&msg);

       
    }
	delete devMan;
	
	return 0;
}
