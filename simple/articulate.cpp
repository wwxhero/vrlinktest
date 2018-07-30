// The following headers are actually already included
// in entitySR.h, but are included here for reference.
#include <vlpi/articulatedPartCollection.h>
#include <vlpi/articulatedPart.h>

// Used for DtInfo in this example
#include <vlutil/vlPrint.h>
#include "articulate.h"

void artPartsExampleInitS(DtEntityStateRepository *anEntity)
{
  DtArticulatedPartCollection* artPartCol = anEntity->artPartList();

   // Here we set some Articulated Parts: A gun with elevation and a turrent with azimuth and rate.

   // The Articulated part collection is a map of DtArticulatedParts keyed to the PartType.
   // Note that the part type is in fact an integer, but normally under DIS
   // you would want the value to be one of the defined enumerations
   // DtArtPartType and DtArtParamType.
   // Note that the getPart() method of DtArticulatedPartCollection will create
   // an articulated part of the specified part type, if it does not exist.
   DtArticulatedPart& turret = artPartCol->getPart(DtPrimaryTurret1);
   DtArticulatedPart& gun = artPartCol->getPart(DtPrimaryGun1);
   turret.setParameter(DtApAzimuth, -0.305f);
   turret.setParameter(DtApAzimuthRate, -0.058f);
   gun.setParameter(DtApElevation, 3.14f);

   // Now, we attach the gun to the turret.  By default, parts are attached to the
   // entity when created.
   artPartCol->attachPart(&gun, &turret);
}

void artPartsExamplePrint(DtEntityStateRepository *anEntity)
{
  DtArticulatedPartCollection* artPartCol = anEntity->artPartList();

   // Read and print all of the articulated parts and parameters.
   // The first loop iterates through the known collections of articulated parts.
   for(DtArticulatedPartCollection::const_iterator artPartIter = artPartCol->begin();
       artPartIter != artPartCol->end();
       ++artPartIter)
   {
      int currentPartType = artPartIter->first;
      DtArticulatedPart* currentPart = artPartIter->second;

      DtInfo << "Part Type: " << DtString(currentPartType) << "\n";

      // Once we have a chosen a DtArticulatedPart, we can retrieve a vector
      // of the parameter metrics set, and iterate through it.
      std::vector<DtArticulatedPart::ParameterMetric> parameterMetrics;
      currentPart->getParameterMetrics(parameterMetrics);

      std::vector<DtArticulatedPart::ParameterMetric>::const_iterator paramMetricIter =
         parameterMetrics.begin();
      std::vector<DtArticulatedPart::ParameterMetric>::const_iterator paramMetricEnd =
         parameterMetrics.end();

      for(;paramMetricIter != paramMetricEnd; ++paramMetricIter)
       {
          int paramMetric = *paramMetricIter;
          DtArticulatedPart::Parameter parameter;

          currentPart->getParameter(paramMetric, parameter);

          DtInfo << "   Param: " << DtString(paramMetric)
                 << "  Value: " << parameter.value << "\n";
       }
   }
}

// This function passes the classes which are needed for Articulated Parts...
void artPartsExampleS(DtEntityStateRepository *anEntity)
{
   artPartsExampleInitS(anEntity);
   artPartsExamplePrint(anEntity);
}