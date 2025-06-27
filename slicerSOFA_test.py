import logging
import os
import qt
import vtk
import random
import time
import uuid
import numpy as np

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer import vtkMRMLMarkupsFiducialNode
from slicer import vtkMRMLMarkupsLineNode
from slicer import vtkMRMLMarkupsNode
from slicer import vtkMRMLMarkupsROINode
from slicer import vtkMRMLModelNode

from SofaEnvironment import Sofa
from SlicerSofa import (
    SlicerSofaWidget,
    SlicerSofaLogic,
    SofaParameterNodeWrapper,
)

from SlicerSofaUtils.Mappings import (
    mrmlModelGridToSofaTetrahedronTopologyContainer,
    mrmlMarkupsFiducialToSofaPointer,
    mrmlMarkupsROIToSofaBoxROI,
    sofaMechanicalObjectToMRMLModelGrid,
    sofaVonMisesStressToMRMLModelGrid,
    arrayFromMarkupsROIPoints,
    arrayVectorFromMarkupsLinePoints,
)

# Load the VTK file as a model
# vtkPath = "/home/zhang/Documents/orbit_SOFA/soft_tissue-models/orbit_soft_vol.vtk"  # Update path if needed
# modelNode = slicer.util.loadModel(vtkPath)

# Set up SOFA scene


import SoftTissueSimulation
logic = SoftTissueSimulation.SoftTissueSimulationLogic()

vtkPath = "/home/chi/Documents/orbit_SOFA/soft_tissue-models/orbit_soft_vol.vtk"  # Update path if needed
simulationModelNode = slicer.util.loadModel(vtkPath)

# Set the layout to 3D view for visualization
layoutManager = slicer.app.layoutManager()
layoutManager.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)

# modelBounds = [0.0] * 6
# simulationModelNode.GetBounds(modelBounds)
#
# # Define the size and center of the ROI (lower third of the model)
# lowerThirdSize = [
#     (modelBounds[1] - modelBounds[0]) * 0.5,
#     (modelBounds[3] - modelBounds[2]) * 0.5,
#     (modelBounds[5] - modelBounds[4]) / 3
# ]
# lowerThirdCenter = [
#     (modelBounds[1] + modelBounds[0]) / 2,
#     (modelBounds[3] + modelBounds[2]) / 2,
#     modelBounds[4] + lowerThirdSize[2] / 2
# ]
#
# # Create and configure the ROI node
# fixedROINode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsROINode", "FixedROI")
# fixedROINode.SetXYZ(lowerThirdCenter)
# fixedROINode.SetRadiusXYZ(*lowerThirdSize)

gravityVectorNode=slicer.util.getNode('gravity')
fixedROINode = slicer.util.getNode('fixedROI')


parameterNode = logic.getParameterNode()
parameterNode.modelNode = simulationModelNode
parameterNode.boundaryROI = fixedROINode
# parameterNode.gravityVector = gravityVectorNode
parameterNode.gravityMagnitude = 100000
parameterNode.dt = 0.01
parameterNode.currentStep = 0
parameterNode.totalSteps = 100

# Start the simulation and render view
logic.startSimulation()
view = slicer.app.layoutManager().threeDWidget(0).threeDView()

# Run simulation steps
for _ in range(parameterNode.totalSteps):
    logic.simulationStep()
    view.forceRender()

# Stop the simulation and clean up
logic.stopSimulation()






#Simulate moving points
# Create and configure the ROI node
fixedROINode = slicer.util.getNode('fixedROI')

# Set initial position of the moving point using logic
startPtNode = slicer.util.getNode('startPt')

parameterNode = logic.getParameterNode()
parameterNode.modelNode = simulationModelNode
# logic.addMovingPoint()
movingPointNode = startPtNode
# movingPointNode = logic.getParameterNode().movingPointNode

# Define start and end positions for the moving point
startPosition = [0, 0, 0]
endPosition = [0, 0, 0]

endPtNode = slicer.util.getNode('endPt')

startPtNode.GetNthControlPointPosition(0, startPosition)
# startPosition = list(movingPointNode.GetNthControlPointPosition(0))
endPtNode.GetNthControlPointPosition(0, endPosition)

# Calculate step size for linear interpolation
totalSteps = 100
interpolationStep = [(end - start) / totalSteps for start, end in zip(startPosition, endPosition)]

# self.delayDisplay("Setting up simulation parameters")
# Assign simulation parameters to the parameter node
parameterNode.boundaryROI = fixedROINode
parameterNode.movingPointNode = movingPointNode
parameterNode.gravityMagnitude = 0  # Disabling gravity
parameterNode.dt = 0.01
parameterNode.currentStep = 0
parameterNode.totalSteps = totalSteps

# self.delayDisplay("Starting moving point-only simulation")
# Start the simulation and render view
logic.startSimulation()
view = slicer.app.layoutManager().threeDWidget(0).threeDView()

# Run simulation steps with gradual movement of the point
for step in range(parameterNode.totalSteps):
    # Update the point's position by adding the interpolation step to the current position
    new_position = [
        startPosition[0] + step * interpolationStep[0],
        startPosition[1] + step * interpolationStep[1],
        startPosition[2] + step * interpolationStep[2]
    ]
    movingPointNode.SetNthControlPointPosition(0, *new_position)

    # Advance simulation and render
    logic.simulationStep()
    view.forceRender()

# Stop the simulation and clean up
logic.stopSimulation()
logic.clean()






def startSimulation():
    """
    Sets up the scene and starts the simulation.
    """
    # TODO: The order here is important. Maybe move part to SlicerSOFA to enforce correct order
    logic.setupMappings()
    setupScene(parameterNode)
    import SlicerSofa
    SlicerSofa.startSimulation()
    logic._simulationRunning = True
    logic.getParameterNode().Modified()


def setupScene(parameterNode):
    """
    Initializes the SOFA simulation scene with parameter and root nodes.

    Args:
        parameterNode: The parameter node containing simulation parameters.
        rootNode (Sofa.Core.Node): The root node of the SOFA scene.

    Raises:
        ValueError: If rootNode is not a valid Sofa.Core.Node.
    """

    # if parameterNode is None:
    #     raise ValueError("parameterNode can't be None")
    # if not getattr(parameterNode, 'sofaParameterNodeWrapped', False):
    #     raise ValueError("parameterNode is not a valid parameterNode wrapped by the sofaParameterNodeWrapper")
    import SlicerSofa
    SLicerSofaLogic = SlicerSofa.SlicerSofaLogic
    SLicerSofaLogic._parameterNode = parameterNode
    #
    # if not isinstance(self._rootNode, Sofa.Core.Node):
    #     raise ValueError("rootNode is not a valid Sofa.Core.Node root node")
    SLicerSofaLogic._rootNode = CreateScene()
    setattr(SLicerSofaLogic._parameterNode, "_rootNode", SLicerSofaLogic._rootNode)
    SLicerSofaLogic.__updateSofa__()
    Sofa.Simulation.init(SLicerSofaLogic._rootNode)
    SLicerSofaLogic._sceneUp = True



# # Map MRML model to SOFA FEM
# mrmlModelGridToSofaTetrahedronTopologyContainer(modelNode, topo)
#
# # # Map deformation back to Slicer for visualization
# sofaMechanicalObjectToMRMLModelGrid(modelNode, femNode.mstate)


# # Simulate and update Slicer
# import Sofa.Simulation
# Sofa.Simulation.init(rootNode)
# for i in range(100):
#     Sofa.Simulation.animate(rootNode, rootNode.dt.value)
#     slicer.util.arrayFromModelPoints(modelNode)[:] = fem.mstate.position.array()
#     slicer.util.arrayFromModelPointsModified(modelNode)


# for i in range(100):
#     Sofa.Simulation.animate(rootNode, rootNode.dt.value)
#     slicerPoints = slicer.util.arrayFromModelPoints(modelNode)
#     sofaPoints = femNode.mstate.position.array()
#     slicerPoints[:] = sofaPoints
#     slicer.util.arrayFromModelPointsModified(modelNode)