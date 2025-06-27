import Sofa
import SofaRuntime
import Sofa.Gui


def main():
    # Make sure to load all SOFA libraries
    SofaRuntime.importPlugin("SofaBaseMechanics")
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("CImgPlugin")
    # SofaRuntime.importPlugin("Sofa.Component.Shell")

    # Call the above function to create the scene graph
    # root.name="root"
    root = Sofa.Core.Node("root")  # root node creation (previously provided by runSofa)
    createScene(root)  # scene graph initialization (p.provided by runSofa)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Find out the supported GUIs
    print("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
    # Launch the GUI (qt or qglviwere)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()
    print("GUI was closed")

    # Run the simulation for 10 steps
    # print("time is = " + str(root.time.value))
    # for iteration in range(10):
    #     print(f'Iteration ${iteration}')
    #     Sofa.Simulation.animate(root, root.dt.value)

    print("Simulation made 10 time steps. Done.")


def createScene(root):
    root.name = "root"
    root.gravity = [0, 0, 0]
    root.dt = 0.01
    # root.addObject('DefaultAnimationLoop', computeBoundingBox=False)

    root.addObject('FreeMotionAnimationLoop')
    root.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")

    confignode = root.addChild("Config")
    confignode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualGrid]
    confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    # For CGLlinearSolver
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative")
    #
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    confignode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.MechanicalLoad")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
    confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D",
                         printLog=False)  # Needed to use components [OglModel]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [UncoupledConstraintCorrection]
    confignode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]

    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]                                                                       onPipeline]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance, DiscreteIntersection]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [RuleBasedContactManager, CollisionResponse]
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]

    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetGeometryAlgorithms TetrahedronSetTopologyContainer
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    # confignode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
    confignode.addObject('RequiredPlugin',
                         name='Sofa.Component.Mapping.Linear')  # Needed to use components [IdentityMapping BarycentricMapping]
    # confignode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection') # Needed to use components [DiscreteIntersection]

    # confignode.addObject('RequiredPlugin', name="Sofa.Component.Shell")

    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    root.addObject("VisualGrid", nbSubdiv=10, size=1000)

    # Collision pipeline
    root.addObject('CollisionPipeline')
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('RuleBasedContactManager', responseParams="mu=" + str(5.0), name='Response',
                   response='FrictionContactConstraint')  # FrictionContactConstraint or PenaltyContact
    root.addObject('LocalMinDistance', alarmDistance=10, contactDistance=1, angleCone=0.01)

    # root.addObject('CollisionResponse', response='FrictionContact', responseParams='mu=0.9')

    # add a mechanical model, so that all our future elements will have the same total mass, volume and inertia matrix
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # # mesh loader for msh
    # meshLoaderCoarse = root.addObject('MeshGmshLoader', name='meshLoaderCoarse',
    #                                   filename='/home/chi/Documents/test_sofa_slicersofa_models/orbital_tissue_down.msh',
    #                                   scale=1.0)
    # meshLoaderFine = root.addObject('MeshOBJLoader', name='meshLoaderFine',
    #                                 filename='/home/chi/Documents/test_sofa_slicersofa_models/orbital_tissue_down.obj',
    #                                 scale=1.0)
    #
    # # Creating the falling sphere object
    # eyeNode = root.addChild("eye")
    #
    # # Creating the falling eyeNode object
    # eyeNode.addObject('EulerImplicitSolver', name='odesolver')
    # eyeNode.addObject('SparseLDLSolver', name='Solver')
    #
    # # tetrahedron topology
    # eyeNode.addObject('TetrahedronSetTopologyContainer', name='topo', src=meshLoaderCoarse.getLinkPath())
    # eyeNode.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3d', name='GeomAlgo')
    #
    # eyeNode.addObject('MechanicalObject', template='Vec3d', name='MechanicalModel', showObject=True)
    # eyeNode.addObject('TetrahedronFEMForceField', name='FEM', youngModulus=1e5, poissonRatio=0.47, method='large')
    # eyeNode.addObject('LinearSolverConstraintCorrection')  # need for gravity filed #LinearSolverConstraint
    # # eyeNode.addObject('UncoupledConstraintCorrection', compliance=1e-9)
    # eyeNode.addObject('MeshMatrixMass', massDensity=1, topology='@topo')
    #
    # collision = eyeNode.addChild('Collision')
    # collision.addObject('MeshTopology', src=meshLoaderFine.getLinkPath())
    # collision.addObject('MechanicalObject', name='CollisionMO')
    # collision.addObject('TriangleCollisionModel', name='CollisionModel')
    # collision.addObject('LineCollisionModel')
    # collision.addObject('PointCollisionModel')
    # collision.addObject('BarycentricMapping', name='CollisionMapping', input='@../MechanicalModel',
    #                     output='@CollisionMO')
    #
    # # Fine visual model for plate with tetrahedron
    # fineVisualModel = eyeNode.addChild('FineVisualModel')
    # fineVisualModel.addObject('OglModel', name='VisualModel', src=meshLoaderFine.getLinkPath())
    # fineVisualModel.addObject('BarycentricMapping', name='Mapping', input='@../MechanicalModel', output='@VisualModel')




    #Load orbit fat model
    orbitFatLoaderCoarse = root.addObject('MeshGmshLoader', name='orbitFatMeshLoaderCoarse',
                                      filename='/home/chi/Documents/test_sofa_slicersofa_models/orbit_fat_down.msh',
                                      scale=1.0)
    orbitFatLoaderFine = root.addObject('MeshOBJLoader', name='orbitFatMeshLoaderFine',
                                    filename='/home/chi/Documents/test_sofa_slicersofa_models/orbit_fat_down.obj',
                                    scale=1.0)

    # Creating the falling sphere object
    orbitFatNode = root.addChild("orbitalFat")

    # Creating the falling eyeNode object
    orbitFatNode.addObject('EulerImplicitSolver', name='odesolver')
    orbitFatNode.addObject('SparseLDLSolver', name='Solver')

    # tetrahedron topology
    orbitFatNode.addObject('TetrahedronSetTopologyContainer', name='topo', src=orbitFatLoaderCoarse.getLinkPath())
    orbitFatNode.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3d', name='GeomAlgo')

    orbitFatNode.addObject('MechanicalObject', template='Vec3d', name='MechanicalModel', showObject=True)
    orbitFatNode.addObject('TetrahedronFEMForceField', name='FEM', youngModulus=0.3, poissonRatio=0.49, method='large')
    orbitFatNode.addObject('LinearSolverConstraintCorrection')  # need for gravity filed #LinearSolverConstraint
    # eyeNode.addObject('UncoupledConstraintCorrection', compliance=1e-9)
    orbitFatNode.addObject('MeshMatrixMass', massDensity=1, topology='@topo')

    orbitFatCollision = orbitFatNode.addChild('Collision')
    orbitFatCollision.addObject('MeshTopology', src=orbitFatLoaderFine.getLinkPath())
    orbitFatCollision.addObject('MechanicalObject', name='CollisionMO')
    orbitFatCollision.addObject('TriangleCollisionModel', name='CollisionModel')
    orbitFatCollision.addObject('LineCollisionModel')
    orbitFatCollision.addObject('PointCollisionModel')
    orbitFatCollision.addObject('BarycentricMapping', name='CollisionMapping', input='@../MechanicalModel',
                        output='@CollisionMO')

    # Fine visual model for plate with tetrahedron
    orbitalFatVisualModel = orbitFatNode.addChild('FineVisualModel')
    orbitalFatVisualModel.addObject('OglModel', name='VisualModel', src=orbitFatLoaderFine.getLinkPath())
    orbitalFatVisualModel.addObject('BarycentricMapping', name='Mapping', input='@../MechanicalModel', output='@VisualModel')



    # orbitLoaderCoarse = root.addObject('MeshGmshLoader', name='orbitLoaderCoarse',
    #                                    filename='/home/chi/Documents/test_sofa_slicersofa_models/left_orbit_down.msh',
    #                                    scale=1.0)
    orbitLoaderFine = root.addObject('MeshOBJLoader', name='orbitLoaderFine',
                                     filename='/home/chvvuments/test_sofa_slicersofa_models/left_orbit_down.obj',
                                     scale=1.0)

    # Creating the orbit object
    orbitNode = root.addChild("Orbit")
    # orbitNode.addObject('EulerImplicitSolver', name='odesolver') #need for gravity field
    # orbitNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05) #need for gravity field
    orbitNode.addObject('MechanicalObject', name="mstate", template="Rigid3", showObject=True)
    orbitNode.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # orbitNode.addObject('UniformMass', name="mass", totalMass=1)
    # orbitNode.addObject('UncoupledConstraintCorrection') #need for gravity filed

    #### Collision subnode for the orbit
    # orbitCollis = orbitNode.addChild('collision_orbit')
    # orbitCollis.addObject('MeshTopology', src=orbitLoaderFine.getLinkPath())
    # orbitCollis.addObject('MechanicalObject', name='CollisionMO')
    # orbitCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    # orbitCollis.addObject('RigidMapping')

    # Fine visual model for plate with tetrahedron
    orbitVisu = orbitNode.addChild('FineVisualModel')
    orbitVisu.addObject('OglModel', name='VisualModel', src=orbitLoaderFine.getLinkPath())
    orbitVisu.addObject('RigidMapping')

    # mesh loader for msh
    # plateMeshLoaderCoarse = root.addObject('MeshGmshLoader', name='plateMeshLoaderCoarse', filename='mesh/plate.msh', scale=1.0)
    # plateMeshLoaderFine = root.addObject('MeshOBJLoader', name='plateMeshLoaderFine', filename='mesh/plate.obj', scale=1.0)


    # Creating the retraction tool object
    # toolMeshLoaderFine = root.addObject('MeshOBJLoader', name='toolMeshLoaderFine',
    #                                     filename='/home/zhang/Documents/orbit_SOFA/soft_tissue-models/tool.obj',
    #                                     scale=1.0)
    # toolNode = root.addChild("tool")
    # toolNode.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=50)

    # Creating the falling toolNode object
    # toolNode.addObject('EulerImplicitSolver', name='odesolver')
    # toolNode.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    #
    # toolNode.addObject('MechanicalObject', template='Rigid3', name='MechanicalModel', showObject=True)
    # toolNode.addObject('UniformMass', name="mass", vertexMass=[10000, volume, inertiaMatrix[:]])
    # toolNode.addObject('UncoupledConstraintCorrection')  # need for gravity filed #LinearSolverConstraint
    #
    # toolCollision = toolNode.addChild('toolCollision')
    # toolCollision.addObject('MeshTopology', src=toolMeshLoaderFine.getLinkPath())
    # toolCollision.addObject('MechanicalObject', name='CollisionMO')
    # toolCollision.addObject('TriangleCollisionModel', name='CollisionModel', contactStiffness='100')
    # toolCollision.addObject('LineCollisionModel')
    # toolCollision.addObject('PointCollisionModel')
    # toolCollision.addObject('RigidMapping')
    #
    # # Fine visual model for plate with tetrahedron
    # toolFineVisualModel = toolNode.addChild('FineVisualModel')
    # toolFineVisualModel.addObject('OglModel', name='VisualModel', src=toolMeshLoaderFine.getLinkPath())
    # toolFineVisualModel.addObject('RigidMapping')

    return root


# if we are in a python environment
if __name__ == '__main__':
    main()

# Class ManageParticles(Sofa.Core.Controller):
# Can create keyboard short cut with customized ForceFiel
# Add and remove particles by creating functions

