# Script: 008_example_1.py
#  - requires SlicerSOFA
#  - requires adjusting path for mesh files

import numpy as np
import Sofa
import Sofa.Core
import Sofa.Simulation

from slicer.util import arrayFromModelPoints
from slicer.util import arrayFromModelPolyIds # For Polydata cells
from SlicerSofaUtils.Mappings import (
    arrayFromModelGridCells,             # To get tetrahedral unstructured grid cells
    arrayFromMarkupsROIPoints,           # To get ROI points from markups ROI node
    sofaMechanicalObjectToMRMLModelGrid  # Transfer mechanical object to unstructured grid
    )

############################################
###### Simulation Hyperparameters
############################################

# Input data parameters
orbital_mesh_file = "/home/chi/Documents/orbit_SOFA/soft_tissue-models/orbit_soft_vol.vtk"
scoop_mesh_file = "/home/chi/Documents/orbit_SOFA/soft_tissue-models/tool.vtk"
roi_file = "/home/chi/Documents/orbit_SOFA/soft_tissue-models/orbital_fixed_ROI.mrk.json"
roi_file2 ="/home/chi/Documents/scoop_fixed_roi.mrk.json"
orbital_mesh_node = None
roi_node = None
sphereNode = None
orbital_mass = 60.0
orbital_youngs_modulus = 1.0 * 1000.0 * 0.001
orbital_poisson_ratio = 0.45
force_vector = np.array([-1000.0, 0.0, 0.0])

# Simulatlon hyperparameters
root_node = None
dt = 0.01

# Simulation control parameters
iteration = 0
iterations = 50
simulating = True

############################################
###### Load Simulation Data
############################################
def loadSimulationData():
    global orbital_mesh_node, roi_node, scoop_mesh_node, roi_node2
    orbital_mesh_node = slicer.util.loadModel(orbital_mesh_file)
    roi_node = slicer.util.loadMarkups(roi_file)
    scoop_mesh_node = slicer.util.loadModel(scoop_mesh_file)
    roi_node2 = slicer.util.loadMarkups(roi_file2)

############################################
###### Create Sofa Scene
############################################

# This simulation scene is largely based on the
# SoftTissueSimulation.py included in SlicerSOFA
def createSofaScene():
    global root_node, roi_node, force_vector

    # Initialize the root node of the SOFA scene
    root_node = Sofa.Core.Node("Root")

    # Initialize main scene headers with necessary plugins for SOFA components
    plugins=["Sofa.Component.IO.Mesh",
             "Sofa.Component.LinearSolver.Direct",
             "Sofa.Component.LinearSolver.Iterative",
             "Sofa.Component.Mapping.Linear",
             "Sofa.Component.Mass",
             "Sofa.Component.ODESolver.Backward",
             "Sofa.Component.Setting",
             "Sofa.Component.SolidMechanics.FEM.Elastic",
             "Sofa.Component.StateContainer",
             "Sofa.Component.Topology.Container.Dynamic",
             "Sofa.GL.Component.Rendering3D",
             "Sofa.Component.AnimationLoop",
             "Sofa.Component.Collision.Detection.Algorithm",
             "Sofa.Component.Collision.Detection.Intersection",
             "Sofa.Component.Collision.Geometry",
             "Sofa.Component.Collision.Response.Contact",
             "Sofa.Component.Constraint.Lagrangian.Solver",
             "Sofa.Component.Constraint.Lagrangian.Correction",
             "Sofa.Component.LinearSystem",
             "Sofa.Component.MechanicalLoad",
             "Sofa.Component.SolidMechanics.Spring",
             "Sofa.Component.Constraint.Lagrangian.Model",
             "Sofa.Component.Mapping.NonLinear",
             "Sofa.Component.Topology.Container.Constant",
             "Sofa.Component.Topology.Mapping",
             "Sofa.Component.Topology.Container.Dynamic",
             "Sofa.Component.Engine.Select",
             "Sofa.Component.Constraint.Projective",
             "MultiThreading",]

    for plugin_name in plugins:
         root_node.addObject("RequiredPlugin", name=plugin_name)

    # Set gravity vector for the simulation (no gravity in this case)

    # with root_node.gravity.writeable() as gravity:
    #     gravity[:] = force_vector.copy()
    root_node.gravity = [0, 100000, 0]

    # Add animation and constraint solver objects to the root node
    root_node.addObject('FreeMotionAnimationLoop', parallelODESolving=True, parallelCollisionDetectionAndFreeMotion=True)
    root_node.addObject('GenericConstraintSolver', maxIterations=10, multithreading=True, tolerance=1.0e-3)

    # Collision pipeline
    root_node.addObject('CollisionPipeline')
    root_node.addObject('ParallelBruteForceBroadPhase')
    root_node.addObject('ParallelBVHNarrowPhase')
    root_node.addObject('CollisionResponse', responseParams="mu=" + str(5.0), name='Response',
                   response='FrictionContactConstraint')  # FrictionContactConstraint or PenaltyContact
    root_node.addObject('LocalMinDistance', alarmDistance=2, contactDistance=0.2)


    # Define a deformable Finite Element Method (FEM) object
    orbitalTissueNode = root_node.addChild('FEM')
    orbitalTissueNode.addObject('EulerImplicitSolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    orbitalTissueNode.addObject('SparseLDLSolver', name="precond", template="CompressedRowSparseMatrixd", parallelInverseProduct=True)
    orbitalTissueNode.addObject('TetrahedronSetTopologyContainer', name="Container",
                      position=slicer.util.arrayFromModelPoints(orbital_mesh_node),
                      tetrahedra=arrayFromModelGridCells(orbital_mesh_node))
    orbitalTissueNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
    orbitalTissueNode.addObject('MechanicalObject', name="mstate", template="Vec3d")
    orbitalTissueNode.addObject('TetrahedronFEMForceField', name="FEM", youngModulus=1000, poissonRatio=0.47, method="large")
    orbitalTissueNode.addObject('MeshMatrixMass', totalMass=1)

    # Add a region of interest (ROI) with fixed constraints in the FEM node
    fixedROI = orbitalTissueNode.addChild('FixedROI')
    fixedROI.addObject('BoxROI', template="Vec3", box=arrayFromMarkupsROIPoints(roi_node), drawBoxes=False,
                       position="@../mstate.rest_position", name="BoxROI",
                       computeTriangles=False, computeTetrahedra=False, computeEdges=False)
    fixedROI.addObject('FixedConstraint', indices="@BoxROI.indices")

    # Set up collision detection within the FEM node
    collisionNode = orbitalTissueNode.addChild('Collision')
    collisionNode.addObject('TriangleSetTopologyContainer', name="Container")
    collisionNode.addObject('TriangleSetTopologyModifier', name="Modifier")
    collisionNode.addObject('Tetra2TriangleTopologicalMapping', input="@../Container", output="@Container")
    collisionNode.addObject('TriangleCollisionModel', name="collisionModel", proximity=0.001, contactStiffness=20)
    collisionNode.addObject('MechanicalObject', name='dofs', rest_position="@../mstate.rest_position")
    collisionNode.addObject('IdentityMapping', name='visualMapping')


    # Apply a linear solver constraint correction in the FEM node
    orbitalTissueNode.addObject('LinearSolverConstraintCorrection', linearSolver="@precond")

    # Scoop
    scoopNode = root_node.addChild('FEM2')
    scoopNode.addObject('EulerImplicitSolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    scoopNode.addObject('SparseLDLSolver', name="precond", template="CompressedRowSparseMatrixd", parallelInverseProduct=True)
    scoopNode.addObject('TetrahedronSetTopologyContainer', name="Container",
                      position=slicer.util.arrayFromModelPoints(scoop_mesh_node),
                      tetrahedra=arrayFromModelGridCells(scoop_mesh_node))
    scoopNode.addObject('TetrahedronSetTopologyModifier', name="Modifier")
    scoopNode.addObject('MechanicalObject', name="mstate", template="Vec3d")
    scoopNode.addObject('TetrahedronFEMForceField', name="FEM", youngModulus=1e10, poissonRatio=0.42, method="large")
    scoopNode.addObject('MeshMatrixMass', totalMass=1)

    # Add a region of interest (ROI) with fixed constraints in the FEM node
    fixedROI2 = scoopNode.addChild('FixedROI2')
    fixedROI2.addObject('BoxROI', template="Vec3", box=arrayFromMarkupsROIPoints(roi_node2), drawBoxes=False,
                       position="@../mstate.rest_position", name="BoxROI",
                       computeTriangles=False, computeTetrahedra=False, computeEdges=False)
    fixedROI2.addObject('FixedConstraint', indices="@BoxROI.indices")

    # Set up collision detection within the FEM node
    scoopCollisionNode = scoopNode.addChild('Collision')
    scoopCollisionNode.addObject('TriangleSetTopologyContainer', name="Container")
    scoopCollisionNode.addObject('TriangleSetTopologyModifier', name="Modifier")
    scoopCollisionNode.addObject('Tetra2TriangleTopologicalMapping', input="@../Container", output="@Container")
    scoopCollisionNode.addObject('TriangleCollisionModel', name="collisionModel", proximity=0.001, contactStiffness=20)
    scoopCollisionNode.addObject('MechanicalObject', name='dofs', rest_position="@../mstate.rest_position")
    scoopCollisionNode.addObject('IdentityMapping', name='visualMapping')

    # Apply a linear solver constraint correction in the FEM node
    scoopNode.addObject('LinearSolverConstraintCorrection', linearSolver="@precond")


    # Define a secpmd deformable Finite Element Method (FEM) object
    # totalMass = 1.0
    # volume = 1.0
    # inertiaMatrix = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    # plateCenterOfMass = [-33.2873, -89.1452, -14.29893, 0, 0, 0, 1]
    # MeshLoaderFine = root_node.addObject('MeshOBJLoader', name='toolMeshLoaderFine',
    #                                     filename='/home/chi/Documents/orbit_SOFA/soft_tissue-models/tool.obj',
    #                                     scale=1.0)
    # scoopNode = root_node.addChild('scoopNode')
    # scoopNode.addObject('EulerImplicitSolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    # scoopNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    # scoopNode.addObject('MechanicalObject', template='Rigid3', name='MechanicalModel',
    #                             position=scoopCenterOfMass, showObject=True, showObjectScale=1.0)
    # scoopNode.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # # scoopNode.addObject('UncoupledConstraintCorrection')
    #
    #
    #
    # # Add a region of interest (ROI) with fixed constraints in the FEM node
    # fixedROI2 = scoopNode.addChild('FixedROI2')
    # fixedROI2.addObject('BoxROI', template="Vec3", box=arrayFromMarkupsROIPoints(roi_node), drawBoxes=False,
    #                    position="@../mstate.rest_position", name="BoxROI",
    #                    computeTriangles=False, computeTetrahedra=False, computeEdges=False)
    # fixedROI2.addObject('FixedConstraint', indices="@BoxROI.indices")
    #
    # # Set up collision detection within the FEM node
    # scoopCollision = scoopNode.addChild('scoopCollision')
    # scoopCollision.addObject('MeshTopology', src=toolMeshLoaderFine.getLinkPath())
    # scoopCollision.addObject('MechanicalObject', name='CollisionMO')
    # scoopCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    # scoopCollision.addObject('LineCollisionModel')
    # scoopCollision.addObject('RigidMapping', globalToLocalCoords=True)
    #
    # # Apply a linear solver constraint correction in the FEM node
    # scoopNode.addObject('LinearSolverConstraintCorrection', linearSolver="@precond")
    #
    # from vtk.util import numpy_support as ns
    #
    # # Convert scoop mesh point data to numpy array
    # scoopPoints = arrayFromModelPoints(scoop_mesh_node)
    #
    # # Use this for visual model (optional)
    # scoopVisual = scoopNode.addChild("ScoopVisual")
    # scoopVisual.addObject("OglModel", name="VisualModel")
    # scoopVisual.addObject("MechanicalObject", name="VisualMO", position=scoopPoints)
    # scoopVisual.addObject("RigidMapping", input="@../MechanicalModel", output="@VisualMO")
    #
    #
    #
    # # # Creating the falling toolNode object
    # # toolNode.addObject('EulerImplicitSolver', name='odesolver')
    # # toolNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    # #
    # # toolMo = toolNode.addObject('MechanicalObject', template='Rigid3', name='MechanicalModel',
    # #                             position=scoopCenterOfMass, showObject=True, showObjectScale=1.0)
    # # toolNode.addObject('UniformMass', name="mass", totalMass=10)
    # # toolNode.addObject('BilateralLagrangianConstraint', template='Rigid3',
    # #                    object1=toolMo.linkpath,
    # #                    object2=toolCMState.linkpath,
    # #                    first_point=0,
    # #                    second_point=0)
    # #
    # # toolNode.addObject('UncoupledConstraintCorrection')  # need for gravity filed #LinearSolverConstraint
    # #
    # # toolCollision = toolNode.addChild('toolCollision')
    # # toolCollision.addObject('MeshTopology', src=toolMeshLoaderFine.getLinkPath())
    # # toolCollision.addObject('MechanicalObject', name='CollisionMO')
    # # toolCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    # # toolCollision.addObject('LineCollisionModel')
    # # toolCollision.addObject('RigidMapping', globalToLocalCoords=True)

    # Initialize the simulation
    Sofa.Simulation.init(root_node)

############################################
###### Update Simulation
############################################
def updateSimulation():
    global iteration, iterations, simulating, root_node, orbital_mesh_node, scoop_mesh_node
    # global iteration, iterations, simulating, root_node, orbital_mesh_node

    Sofa.Simulation.animate(root_node, root_node.dt.value)

    # Transfer new coordinates from sofa mechanical object to slicer MRML model node
    sofaMechanicalObjectToMRMLModelGrid(orbital_mesh_node, root_node['FEM.mstate'])

    # This is needed to notify slicer that changes to the mesh have occurred
    slicer.util.arrayFromModelPointsModified(orbital_mesh_node)

    sofaMechanicalObjectToMRMLModelGrid(scoop_mesh_node, root_node['FEM2.mstate'])
    slicer.util.arrayFromModelPointsModified(scoop_mesh_node)

    # sofaRigidState = root_node['scoopNode.MechanicalModel'].position.array()[0][:3]
    # scoopPoints = arrayFromModelPoints(scoop_mesh_node)
    # scoopPoints[:] += sofaRigidState  # naive translation-only update
    # slicer.util.arrayFromModelPointsModified(scoop_mesh_node)

    # iteration management
    iteration += 1
    simulating = iteration < iterations
    if iteration % 10 == 0:
        print(f"Iteration {iteration}")
    if simulating:
        qt.QTimer.singleShot(10, updateSimulation)
    else:
        print("Simlation stopped")

############################################
###### Execution flow
############################################

# This will clear the scene, in case we want to
# load the script over and over in the same Slicer instance
slicer.mrmlScene.Clear()

loadSimulationData()
createSofaScene()
updateSimulation()
