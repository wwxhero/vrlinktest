/****************************************************************************
 * Copyright (c) 2016 VT MAK
 * All rights reserved.
 ****************************************************************************/

//! \file talk.cxx
//! \brief Contains the simple talk example.
//! \ingroup simple

#include <vl/exerciseConn.h>
#include <vl/exerciseConnInitializer.h>
#include <vl/topoView.h>
#include <vl/entityPublisher.h>
#include <vl/entityStateRepository.h>
#include <vl/fireInteraction.h>
#include <vl/iffPublisher.h>
#include <vlutil/vlMiniDumper.h>

#include <vlpi/entityTypes.h>

#include <vlutil/vlProcessControl.h>

#include <iostream>

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

int main( int argc, char* argv[] )
{
   // Used for error handling
   DtINIT_MINIDUMPER( "Talk" );

   try
   {
      // Create a connection to the exercise or federation execution.
      DtVrlApplicationInitializer appInit( argc, argv, "VR-Link Talk" );

#if DtDIS
      // Change some defaults. Please note that this is DIS specific code.
      // Notice how this is the only line of DIS specific code in this example,
      // everything else is using the protocol independent API.
      appInit.setUseAsynchIO( true );
      appInit.setDisVersionToSend( 7 );
#else
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

      DtExerciseConn::InitializationStatus status = DtExerciseConn::DtINIT_SUCCESS;

      // The DtExerciseConn instance is perhaps the most important class
      // in any VR-Link exercise. Think of it as the hub which holds
      // everything else together. Technically this instance is what's
      // responsible for connecting this federate to another federate via
      // a network, or an RTI. After exConn is created, and if no error
      // has occurred this simulator will be a live federate.
      DtExerciseConn exConn(appInit, &status);

      if (status != 0)
      {
         std::cout << "Error creating exercise connection." << std::endl;
         return -1;
      }

      DtEntityType f18Type(DtPlatform, DtPlatformDomainAir,
                           DtUnitedStates, DtFighter, DtF18, 0, 0);

      // Create an entity publisher for the entity we are simulating.
      DtEntityPublisher entityPub(f18Type, &exConn,
                                  DtDrDrmRvw, DtForceFriendly,
                                  DtEntityPublisher::guiseSameAsType());

      // Hold on to the entity's state repository, where we can set data.
      DtEntityStateRepository* esr = entityPub.entityStateRep();

      // Create a topographic view on the state repository, so we
      // can set position information in topographic coordinates.
      double refLatitude  = DtDeg2Rad(   35.699760 );
      double refLongitude = DtDeg2Rad( -121.326577 );
      DtTopoView topoView(esr, refLatitude, refLongitude);

      // We can use the ESR to set state.
      esr->setMarkingText("VR-Link");
      topoView.setOrientation(DtTaitBryan(0.0, 0.0, 0.0));

      // Initialize VR-Link time.
      DtClock* clock = exConn.clock();

      DtVector position(0, 0, -100);
      DtVector32 velocity(20, 0, 0);

      //Shutdown handler in order to smoothly end if the HLA exercise shuts down
      int forever = 1;
#ifdef DtHLA
      exConn. addRtiErrorCb(&DtRtiShutdownHandler, &forever);
#endif

      // Send a Fire Interaction.
      DtFireInteraction fire;
      fire.setAttackerId(entityPub.globalId());
      exConn.sendStamped(fire);

      // Main loop
      DtTime dt = 0.05;
      DtTime simTime = 0;
      clock->init();
      while (forever && simTime <= 10.0)
      {
         // Tell VR-Link the current value of simulation time.
         clock->setSimTime(simTime);

         // Process any incoming messages.
         exConn.drainInput();

         // Set the current position information.
         topoView.setLocation(position);
         topoView.setVelocity(velocity);

         // Call tick, which insures that any data that needs to be
         // updated is sent.
         entityPub.tick();

         // Set up for next iteration.
         position[0] += velocity[0] * dt;

         simTime     += dt;

         // Wait till real time equals simulation time of next step.
         DtSleep(simTime - clock->elapsedRealTime());
      }
   }
   DtCATCH_AND_WARN(std::cout);
   return 0;
}
