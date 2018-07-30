/****************************************************************************
 * Copyright (c) 2016 VT MAK
 * All rights reserved.
 ****************************************************************************/

//! \file listen.cxx
//! \brief Contains the simple listen example.
//! \ingroup simple

#include <vl/exerciseConn.h>
#include <vl/exerciseConnInitializer.h>

#include <vlutil/vlProcessControl.h>
#include <vlutil/vlMiniDumper.h>

#include <vl/reflectedEntityList.h>
#include <vl/entityStateRepository.h>
#include <vl/reflectedEntity.h>
#include <vl/fireInteraction.h>
#include <vl/topoView.h>
#include <vl/fomMapper.h>
#include <iostream>
#include <vl/environmentProcessRepository.h>
#include <vl/reflectedEnvironmentProcess.h>
#include <vl/reflectedEnvironmentProcessList.h>
#include "adaptation.h"
#include "Clock.h"
#include "articulate.h"

int keybrdTick(void);


void DtRtiShutdownHandler(const char * label, void* finished)
{
	//This callback may occur multiple times, since it gets called
	//every time we make an RTI call with an error. So, we mark this
	//as finished and make sure that we only signal once.
	int* finishedInt = (int*)finished;
	if (*finishedInt)
	{
		DtWarn << "Shutting down: " << label << std::endl;
	}
	*finishedInt = 0; //Order a shutdown;
}

// A callback for when a DtReflectedEnvironmentProcess object is updated.
void printEnvProc(DtReflectedEnvironmentProcess* obj, void* userData)
{
	obj->epsr()->printData();
}

// A callback for when a new DtReflectedEnvironmentProcess object is added.
void envProcAddedCb(DtReflectedEnvironmentProcess* obj, void* userData)
{
	// register a callback for when an existing object is updated.
	obj->addPostUpdateCallback(printEnvProc, userData);
}



// Define a callback to process fire interactions.
void fireCb(DtFireInteraction* fire, void* /*usr*/)
{
	std::cout << "Fire Interaction from "
		<<  fire->attackerId().string() << std::endl;
}

int main(int argc, char** argv)
{
	DtINIT_MINIDUMPER( "Listen" );

	try
	{

		// Create an exercise conn initializer. This will parse the command
		// line, as well as parse an mtl file with the same name as this application.
		DtVrlApplicationInitializer appInit(argc, argv, "VR-Link Listen");

#if DtDIS
		// Please note, this is the only protocol specific code in this example. For
		// HLA there is no network connection, so there is no concept of AsyncIO at the
		// VR-Link level.
		appInit.setUseAsynchIO( true );
		appInit.setDisVersionToSend( 7 );
#else
		// Set the default FED File, executable, and version
		appInit.setFedFileName( DtDefaultRpr2Fom );
		appInit.setExecName( "MAK-RPR-2.0" );
		appInit.setRprFomVersion( 2.0 );
		appInit.setRprFomRevision( 1 );
#ifdef DtHLA_1516_EVOLVED
		std::vector<DtString> fomModules;
		std::vector<DtString> fomModuleFromCommandLine;
		appInit.fomModules(fomModuleFromCommandLine);
		fomModules.push_back("MAK-VRFExt-1_evolved.xml");
		fomModules.push_back("MAK-DIGuy-2_evolved.xml");
		fomModules.push_back("MAK-LgrControl-1_evolved.xml");
		fomModules.push_back("MAK-VRFAggregate-1_evolved.xml");
		fomModules.push_back("MAK-DynamicTerrain-1_evolved.xml");
		fomModules.insert(fomModules.end(), fomModuleFromCommandLine.begin(), fomModuleFromCommandLine.end());
		appInit.setFomModules(fomModules);
#endif
#endif

		// Process the command line. This will actually set values in the
		// DtVrlApplicationInitializer class and should be called before
		// creating the DtExerciseConn instance.
		appInit.parseCmdLine();
		appInit.setTimeStampType(DtTimeStampAbsolute);

		DtExerciseConn::InitializationStatus status = DtExerciseConn::DtINIT_SUCCESS;

		// The DtExerciseConn instance is perhaps the most important class
		// in any VR-Link exercise. Think of it as the hub which holds
		// everything else together. Technically this instance is what's
		// responsible for connecting this federate to another federate via
		// a network, or an RTI. After exConn is created, and if no error
		// has occurred this simulator will be a live federate.
		DtExerciseConn exConn(appInit, &status);
		//DtTimeStampType t = exConn.timeStampType();

		if (status != 0)
		{
			std::cout << "Error creating exercise connection." << std::endl;
			return -1;
		}

		DtReflectedEnvironmentProcessList relEnv(&exConn);
		relEnv.addEnvironmentProcessAdditionCallback(envProcAddedCb, NULL);

		// Register a callback to handle fire interactions.
		DtFireInteraction::addCallback(&exConn, fireCb, NULL);

		// Create an object to manage entities that we hear about
		// on the network.
		DtReflectedEntityList rel(&exConn);

		// Initialize VR-Link time.
		DtClock* clock = exConn.clock();

		//Shutdown handler in order to smoothly end if the HLA exercise shuts down
		int forever = 1;
#ifdef DtHLA
		exConn.addRtiErrorCb(&DtRtiShutdownHandler, &forever);
#endif
		CClockStaticAln sysClock;
		sysClock.StartClock();
		clock->init();
		DtTime dt = 1.0/60.0; //60 hz
		DtTime simTimeO = (DtTime)sysClock.GetTickCnt()/(DtTime)1000; //in seconds
		DWORD time_n = GetTickCount();

		bool bTimeOver = false;
		while (forever && !bTimeOver)
		{
			// Check if user hit 'q' to quit.
			if (keybrdTick() == -1)
				break;

			//simTime = clock->elapsedRealTime();
			// Tell VR-Link the current value of simulation time.
			DtTime simTime = (DtTime)sysClock.GetTickCnt()/(DtTime)1000; //in seconds
			clock->setSimTime(simTime);

			// Process any incoming messages.
			exConn.drainInput();

			// Find the first entity in the reflected entity list.
			DtReflectedEntity *first = rel.first();

			if (first)
			{
				// Grab its state repository, where we can inspect its data.
				DtEntityStateRepository *esr = first->entityStateRep();
				esr->setAlgorithm(c_drkDefault);
				esr->useSmoother();
				artPartsExamplePrint(esr);
				// Create a topographic view on the state repository, so we
				// can look at position information in topographic coordinates.
				double refLatitude  = DtDeg2Rad(  35.699760);
				double refLongitude = DtDeg2Rad(-121.326577);
				DtTopoView topoView(esr, refLatitude, refLongitude);

				ExternalDriverStateTran stateTran;

				stateTran.vel = topoView.velocity();
				stateTran.acc = topoView.acceleration();
				stateTran.ori = topoView.orientation();
				stateTran.rot = topoView.rotationalVelocity();
				stateTran.loc = topoView.location();
				DWORD time_np = GetTickCount();
				Logout(simTime, time_np - time_n, stateTran);

				ExternalDriverState stateSim;
				Transform(stateTran, stateSim);
				Logout(simTime, time_np - time_n, stateSim);

				time_n = time_np;
				// Print the position.
				// Since it returns a DtString, we need to force it to const char*
				// with a cast.
				//std::cout << "Position of first entity: "
				//   << topoView.location().string() << " time: " << simTime << std::endl;

			}
			DtTime elapse = simTime - simTimeO;
			// Sleep till next iteration.
			DtSleep(elapse + dt - clock->elapsedRealTime());
			bTimeOver = (elapse > 600);
		}
	}
	DtCATCH_AND_WARN(std::cout);
	return 0;
}

int keybrdTick()
{
	char *keyPtr = DtPollBlockingInputLine();
	if (keyPtr && (*keyPtr == 'q' || *keyPtr == 'Q'))
		return -1;
	else
		return 0;
}


