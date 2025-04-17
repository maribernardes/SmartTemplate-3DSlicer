import logging
import os

import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from PythonQt import QtCore, QtGui
import vtk.util.numpy_support as numpy_support
import xml.etree.ElementTree as ET


# import SimpleITK as sitk
# import sitkUtils
import numpy as np
from time import sleep
import datetime

class SmartTemplate(ScriptedLoadableModule):

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "SmartTemplate"
    self.parent.categories = ["IGT"] 
    self.parent.dependencies = ["ZFrameRegistration", "ROS2"]  # TODO: add here list of module names that this module requires
    self.parent.contributors = ["Mariana Bernardes (BWH)"] 
    self.parent.helpText = """ This module is used to interact with the ROS2 packages for SmartTemplate needle guide robot. Uses ZFrameRegistration module for initialization of the ZTransform and SlicerROS2 module"""
    # TODO: replace with organization, grant and thanks
    self.parent.acknowledgementText = """ """

    # Additional initialization step after application startup is complete
    # TODO: include sample data and testing routines
    # slicer.app.connect("startupCompleted()", registerSampleData)

################################################################################################################################################
# Widget Class
################################################################################################################################################

class SmartTemplateWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
  """
  # Called when the user opens the module the first time and the widget is initialized.
  def __init__(self, parent=None):
    ScriptedLoadableModuleWidget.__init__(self, parent)
    VTKObservationMixin.__init__(self)  # needed for parameter node observation
    self.logic = None
    self._parameterNode = None
    self._updatingGUIFromParameterNode = False

  ### Widget setup ###################################################################
  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)
    print('Widget Setup')
    ####################################
    ##                                ##
    ## UI Components                  ##
    ##                                ##
    ####################################

    ## Robot collapsible button            
    ####################################
    robotCollapsibleButton = ctk.ctkCollapsibleButton()
    robotCollapsibleButton.text = 'Robot'
    self.layout.addWidget(robotCollapsibleButton)

    # Load robot button
    robotFormLayout = qt.QFormLayout(robotCollapsibleButton)
    robotFormLayout.addRow('', qt.QLabel(''))  # Vertical space
    self.loadButton = qt.QPushButton('Load SmartTemplate')
    self.loadButton.toolTip = 'Loads robot in 3DSlicer'
    self.loadButton.enabled = True
    robotFormLayout.addRow(self.loadButton)

    # ZTransform matrix
    registrationLayout = qt.QHBoxLayout()
    self.zTransformSelector = slicer.qMRMLNodeComboBox()
    self.zTransformSelector.nodeTypes = ['vtkMRMLLinearTransformNode']
    self.zTransformSelector.selectNodeUponCreation = True
    self.zTransformSelector.addEnabled = True
    self.zTransformSelector.removeEnabled = True
    self.zTransformSelector.noneEnabled = True
    self.zTransformSelector.showHidden = False
    self.zTransformSelector.showChildNodeTypes = False
    self.zTransformSelector.setMRMLScene(slicer.mrmlScene)
    self.zTransformSelector.setToolTip('Select the ZFrameToScanner Transform')
    ZTransformLabel = qt.QLabel('ZTransform:')
    registrationLayout.addWidget(ZTransformLabel)
    registrationLayout.addWidget(self.zTransformSelector)
    
    # Register robot button 
    self.registerButton = qt.QPushButton('Register SmartTemplate')
    self.registerButton.toolTip = 'Registers robot to scanner'
    self.registerButton.enabled = False
    registrationLayout.addWidget(self.registerButton)
    robotFormLayout.addRow(registrationLayout)

    jointsContainer = qt.QWidget()
    jointsContainer.setMinimumHeight(200)  # Set minimum height for the container
    # Robot joints
    self.jointsLayout = qt.QVBoxLayout(jointsContainer)
    robotFormLayout.addWidget(jointsContainer)

    # Robot position
    positionLayout = qt.QHBoxLayout()
    positionLabel = qt.QLabel('Position:')
    positionLabel.setFixedWidth(50)
    self.positionRASTextbox = qt.QLineEdit('(---, ---, ---) RAS')
    self.positionRASTextbox.setFixedWidth(180)
    self.positionRASTextbox.setReadOnly(True)
    self.positionRASTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRASTextbox.toolTip = 'Robot current position (scanner frame)'
    self.positionRobotTextbox = qt.QLineEdit('(---, ---, ---) XYZ')
    self.positionRobotTextbox.setFixedWidth(180)
    self.positionRobotTextbox.setReadOnly(True)
    self.positionRobotTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRobotTextbox.toolTip = 'Robot current position (robot frame)'
    positionLayout.addWidget(positionLabel)
    positionLayout.addWidget(self.positionRASTextbox)
    positionLayout.addWidget(self.positionRobotTextbox)
    timestampLabel = qt.QLabel('Timestamp:')
    timestampLabel.setFixedWidth(70)
    self.timeStampTextbox = qt.QLineEdit('-- : -- : --.----')
    self.timeStampTextbox.setFixedWidth(120)
    self.timeStampTextbox.setReadOnly(True)
    self.timeStampTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.timeStampTextbox.toolTip = 'Timestamp from last shape measurement'
    positionLayout.addWidget(timestampLabel)
    positionLayout.addWidget(self.timeStampTextbox)
    robotFormLayout.addRow(positionLayout)


    ## Planning collapsible button                 
    ####################################
    
    planningCollapsibleButton = ctk.ctkCollapsibleButton()
    planningCollapsibleButton.text = 'Planning'
    self.layout.addWidget(planningCollapsibleButton)   
    planningFormLayout = qt.QVBoxLayout(planningCollapsibleButton)
    
    # Points selection
    markupsLayout = qt.QFormLayout()
    planningFormLayout.addLayout(markupsLayout)
    self.pointListSelector = slicer.qSlicerSimpleMarkupsWidget()    
    self.pointListSelector.setMRMLScene(slicer.mrmlScene)
    self.pointListSelector.setNodeSelectorVisible(False)
    self.pointListSelector.markupsPlaceWidget().setPlaceMultipleMarkups(True)
    self.pointListSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.pointListSelector.setMaximumHeight(90)
    self.pointListSelector.tableWidget().show()
    self.pointListSelector.toolTip = 'Select 2 points: TARGET and ENTRY'
    # self.pointListSelector.markupsPlaceWidget().setPlaceModePersistency(True)
    markupsLayout.addRow('Planned points:', self.pointListSelector)
    
    # Adjust Entry button 
    self.adjustEntryButton = qt.QPushButton('Adjust ENTRY to TARGET')
    self.adjustEntryButton.toolTip = 'Adjust ENTRY to have it aligned to TARGET'
    self.adjustEntryButton.enabled = False
    markupsLayout.addRow('', self.adjustEntryButton)
    markupsLayout.addRow('', qt.QLabel(''))  # Vertical space

    commandButtonsLayout = qt.QHBoxLayout()
    self.homeButton = qt.QPushButton('HOME')
    self.homeButton.toolTip = 'Go to robot HOME position'
    self.homeButton.enabled = False
    commandButtonsLayout.addWidget(self.homeButton)

    self.retractButton = qt.QPushButton('RETRACT')
    self.retractButton.toolTip = 'Fully retract needle'
    self.retractButton.enabled = False
    commandButtonsLayout.addWidget(self.retractButton)
    
    self.alignButton = qt.QPushButton('ALIGN')
    self.alignButton.toolTip = 'Align robot for insertion'
    self.alignButton.enabled = False
    commandButtonsLayout.addWidget(self.alignButton)

    self.approachButton = qt.QPushButton('APPROACH')
    self.approachButton.toolTip = 'Approach robot to skin'
    self.approachButton.enabled = False
    commandButtonsLayout.addWidget(self.approachButton)
    markupsLayout.addRow(commandButtonsLayout)

## Insertion collapsible button                
    ####################################
    
    insertionCollapsibleButton = ctk.ctkCollapsibleButton()
    insertionCollapsibleButton.text = 'Insertion'    
    self.layout.addWidget(insertionCollapsibleButton)
    insertionFormLayout = qt.QFormLayout(insertionCollapsibleButton)


    insertionFormLayout.addRow('', qt.QLabel(''))  # Vertical space
    insertionButtonsLayout = qt.QHBoxLayout()
    self.toTargetButton= qt.QPushButton('Insert to target')
    self.toTargetButton.toolTip = 'Insert the needle to target'
    self.toTargetButton.enabled = False
    insertionButtonsLayout.addWidget(self.toTargetButton)

    self.stepButton= qt.QPushButton('Insert length')
    self.stepButton.toolTip = 'Insert the needle stepwise a certain lenght'
    self.stepButton.enabled = True
    insertionButtonsLayout.addWidget(self.stepButton)

    self.stepSizeTextbox = qt.QLineEdit('20.0')
    self.stepSizeTextbox.setReadOnly(False)
    self.stepSizeTextbox.setMaximumWidth(50)
    insertionButtonsLayout.addWidget(self.stepSizeTextbox)
    insertionButtonsLayout.addWidget(qt.QLabel('mm'))
    insertionFormLayout.addRow(insertionButtonsLayout)

    self.layout.addStretch(1)
    
    ####################################
    ##                                ##
    ## UI Behavior                    ##
    ##                                ##
    ####################################

    # Internal variables
    self.isRobotLoaded = False  # Is the robot loaded?
    self.jointNames = None
    self.jointLimits = None
    self.jointValues = None
    
    # Initialize module logic
    self.logic = SmartTemplateLogic()

    # Make sure parameter node is initialized (needed for module reload)
    self.initializeParameterNode()
        
    # These connections ensure that we update parameter node when scene is closed
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
    self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

    # These connections ensure that we synch robot info with the logic
    self.addObserver(self.logic.jointValues, vtk.vtkCommand.ModifiedEvent, self.onJointsChange)
    self.addObserver(self.logic.robotPositionMarkupsNode, vtk.vtkCommand.ModifiedEvent, self.onPositionChange)

    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.zTransformSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.updateParameterNodeFromGUI)
    
    # Connect Qt widgets to event calls
    self.loadButton.connect('clicked(bool)', self.loadRobot)
    self.registerButton.connect('clicked(bool)', self.registerRobot)
    self.adjustEntryButton.connect('clicked(bool)', self.adjustEntry)
    self.retractButton.connect('clicked(bool)', self.retractNeedle)
    self.homeButton.connect('clicked(bool)', self.homeRobot)
    self.alignButton.connect('clicked(bool)', self.alignForInsertion)
    self.approachButton.connect('clicked(bool)', self.approachSkin)
    self.toTargetButton.connect('clicked(bool)', self.toTarget)
    self.stepButton.connect('clicked(bool)', self.insertStep)
    self.pointListSelector.connect('updateFinished()', self.onPointListChanged)

    # Initialize widget variables and updateGUI
    self.updateGUI()
    self.pointListSelector.setCurrentNode(self.logic.pointListNode)
    interactionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLInteractionNodeSingleton")
    interactionNode.SetCurrentInteractionMode(interactionNode.ViewTransform)  # Instead of interactionNode.Place


  ### Widget functions ###################################################################
  # Called when the application closes and the module widget is destroyed.
  def cleanup(self):
    self.removeObservers()

  # Called each time the user opens this module.
  # Make sure parameter node exists and observed
  def enter(self):
    self.initializeParameterNode() 

  # Called each time the user opens a different module.
  # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
  def exit(self):
    self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

  # Called just before the scene is closed.
  # Parameter node will be reset, do not use it anymore
  def onSceneStartClose(self, caller, event):
    self.logic.closeConnection()    
    self.setParameterNode(None)

  # Called just after the scene is closed.
  # If this module is shown while the scene is closed then recreate a new parameter node immediately
  def onSceneEndClose(self, caller, event):
    if self.parent.isEntered:
      self.initializeParameterNode()
        
  # Ensure parameter node exists and observed
  # Parameter node stores all user choices in parameter values, node selections, etc.
  # so that when the scene is saved and reloaded, these settings are restored.
  def initializeParameterNode(self):
    # Load default parameters in logic module
    self.setParameterNode(self.logic.getParameterNode())

  # Set and observe parameter node.
  # Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
  def setParameterNode(self, inputParameterNode):
    if inputParameterNode:
      self.logic.setDefaultParameters(inputParameterNode)
    # Unobserve previously selected parameter node and add an observer to the newly selected.
    # Changes of parameter node are observed so that whenever parameters are changed by a script or any other module
    # those are reflected immediately in the GUI.
    if self._parameterNode is not None and self.hasObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode):
        self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    self._parameterNode = inputParameterNode
    if self._parameterNode is not None:
        self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)
    # Initial GUI update
    self.updateGUIFromParameterNode()

  # This method is called whenever parameter node is changed.
  # The module GUI is updated to show the current state of the parameter node.
  def updateGUIFromParameterNode(self, caller=None, event=None):
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Make sure GUI changes do not call updateParameterNodeFromGUI (it could cause infinite loop)
    self._updatingGUIFromParameterNode = True
    # Update node selectors and input boxes and sliders
    self.zTransformSelector.setCurrentNode(self._parameterNode.GetNodeReference('ZTransform'))
    self.pointListSelector.setCurrentNode(self._parameterNode.GetNodeReference('Planning'))
    self.isRobotLoaded = self._parameterNode.GetParameter('RobotLoaded') == 'True'
    self.updateGUI()
    # All the GUI updates are done
    self._updatingGUIFromParameterNode = False

  # This method is called when the user makes any change in the GUI.
  # The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
  def updateParameterNodeFromGUI(self, caller=None, event=None):
    if self._parameterNode is None or self._updatingGUIFromParameterNode:
      return
    # Modify all properties in a single batch
    wasModified = self._parameterNode.StartModify()  
    # Update paramenters_nodes
    self._parameterNode.SetNodeReferenceID('ZTransform', self.zTransformSelector.currentNodeID)
    # All paramenter_nodes updates are done
    self._parameterNode.EndModify(wasModified)
    
  # Called when the Point List is updated
  def onPointListChanged(self):    
    self.logic.addPlanningPoint()
    self.updateGUI()
  
  # Update the connection statusLabel  
  def onConnectionStatusChange(self, caller=None, event=None):
    self.updateGUI()

  # Update GUI buttons, markupWidget and connection statusLabel
  def updateGUI(self):
    # Check status
    robotRegistered = False if self.logic.mat_RobotToScanner is None else True
    zFrameSelected = (self.zTransformSelector.currentNode() is not None)
    pointsSelected = (self.logic.getNumberOfPoints() == 2)
    if (pointsSelected and zFrameSelected and robotRegistered):
      robotAligned = self.logic.isRobotAligned()
    else:
      robotAligned = False

    # Update buttons accordingly
    self.loadButton.enabled = not self.isRobotLoaded
    self.registerButton.enabled = zFrameSelected and self.isRobotLoaded
    self.adjustEntryButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.homeButton.enabled = self.isRobotLoaded
    self.retractButton.enabled = self.isRobotLoaded
    self.alignButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.approachButton.enabled = robotAligned and pointsSelected and robotRegistered and self.isRobotLoaded
    
    # Add joints if not already added:
    if self.jointNames is None and self.isRobotLoaded:
      self.addJoints()
    # self.pointListSelector.placeActive(not pointsSelected)

  def format3DPoint(self, point, frame=None, decimals=2):
    formatted = f"({point[0]:.{decimals}f}, {point[1]:.{decimals}f}, {point[2]:.{decimals}f})"
    if frame:
        formatted += f" {frame}"
    return formatted

  def addJoints(self):
    (self.jointNames, self.jointLimits) = self.logic.getDefinedJoints()
    if self.jointNames is None:
      print('Could not load joints')
    else:
      # Initialize joint values with 0.0 
      self.jointValues = {joint_name: 0.0 for joint_name in self.jointNames}
      # Dictionaries to hold widgets
      self.sliders = {}
      self.text_boxes = {}
      self.current_value_boxes = {}  # For 'Current [mm]' textboxes
      for joint in self.jointNames:
          # Remove '_joint' suffix and capitalize the label
          joint_label = qt.QLabel(f'{joint.replace("_joint", "").capitalize()}')
          # Slider for current joint state
          slider = qt.QSlider(qt.Qt.Horizontal)
          limits = self.jointLimits[joint]
          slider.setMinimum(int(limits['min']))
          slider.setMaximum(int(limits['max']))
          slider.setEnabled(False)  # Non-editable
          slider.setValue(self.jointValues[joint])
          # Min and Max labels
          min_label = qt.QLabel(f"{limits['min']}")
          max_label = qt.QLabel(f"{limits['max']}")
          # Layout for slider and labels
          slider_layout = qt.QHBoxLayout()
          slider_layout.addWidget(min_label)
          slider_layout.addWidget(slider)
          slider_layout.addWidget(max_label)
          # Text box for current joint value (non-editable)
          current_value_box = qt.QLineEdit('0.0')
          current_value_box.setReadOnly(True)
          current_value_box.setStyleSheet("background: transparent; border: none; color: black;")
          current_value_box.setText(str(self.jointValues[joint]))
          current_value_label = qt.QLabel('Current [mm]:')
          # Layout for current joint values
          value_layout = qt.QHBoxLayout()
          value_layout.addWidget(current_value_label)
          value_layout.addWidget(current_value_box)
          # Layout for each joint
          joint_layout = qt.QVBoxLayout()
          joint_layout.addWidget(joint_label)
          joint_layout.addLayout(slider_layout)
          joint_layout.addLayout(value_layout)
          # Add the joint layout to the main joint controls layout
          self.jointsLayout.addLayout(joint_layout)
          # Add a horizontal separator line after each joint section
          separator_line = qt.QFrame()
          separator_line.setFrameShape(qt.QFrame.HLine)
          separator_line.setFrameShadow(qt.QFrame.Sunken)
          self.jointsLayout.addWidget(separator_line)
          # Save references
          self.sliders[joint] = slider
          self.current_value_boxes[joint] = current_value_box


  # Update sliders and text boxes with current joint values
  def onJointsChange(self, caller=None, event=None):
    if self.jointNames is None:
      return
    for joint in self.jointNames:
      if caller.GetAttribute(joint) is None:
        return
      self.jointValues[joint] = float(caller.GetAttribute(joint))
      self.sliders[joint].setValue(self.jointValues[joint])
      self.current_value_boxes[joint].setText(f"{self.jointValues[joint]:.2f}")

  # Update text boxes with current position values
  def onPositionChange(self, caller=None, event=None):
    robotPositionXYZ = self.logic.getRobotPosition()
    #print('Time: %s / Tip (RAS): %s' %(timestamp,tipCoordinates))
    if robotPositionXYZ is not None:
      self.positionRobotTextbox.setText(self.format3DPoint(robotPositionXYZ, 'XYZ'))
    else:
      self.positionRobotTextbox.setText('(---, ---, ---)')
    robotPositionRAS = self.logic.getRobotPosition('world')
    if robotPositionRAS is not None:
      self.positionRASTextbox.setText(self.format3DPoint(robotPositionRAS, 'RAS'))
    else:
      self.positionRASTextbox.setText('(---, ---, ---)')
    self.timeStampTextbox.setText(self.logic.getTimestamp())
    self.updateGUI()

  def loadRobot(self):
    print('UI: loadRobot()')
    self.logic.loadRobot()
    self.updateGUI()

  def registerRobot(self):
    print('UI: registerRobot()')
    zTransformNode = self.zTransformSelector.currentNode()
    self.logic.registerRobot(zTransformNode)
    self.updateGUI()

  def adjustEntry(self):
    print('UI: adjustEntry()')
    # Get current transform node
    pointListNode = self.pointListSelector.currentNode()
    self.logic.adjustEntry()
    self.updateGUI()

  def retractNeedle(self):
    print('UI: retractNeedle()')
    self.logic.sendDesiredCommand('RETRACT')
    self.updateGUI()

  def homeRobot(self):
    print('UI: homeRobot()')
    self.logic.sendDesiredCommand('HOME')
    self.updateGUI()

  def alignForInsertion(self):
    print('UI: alignForInsertion()')
    self.logic.alignForInsertion()
    self.updateGUI()

  def approachSkin(self):
    print('UI: approachSkin()')
    self.logic.approachSkin()
    self.updateGUI()

  def toTarget(self):
    print('UI: toTarget()')
    self.updateGUI()

  def insertStep(self):
    print('UI: insertStep()')
    self.logic.insertStep(float(self.stepSizeTextbox.text.strip()))
    self.updateGUI()    
################################################################################################################################################
# Logic Class
################################################################################################################################################

class SmartTemplateLogic(ScriptedLoadableModuleLogic):

  def __init__(self):
    ScriptedLoadableModuleLogic.__init__(self)
    self.cliParamNode = None
    print('Logic: __init__')

    # SlicerROS2 module internal variables
    self.rosLogic = slicer.util.getModuleLogic('ROS2')
    self.rosNode = self.rosLogic.GetDefaultROS2Node()
    self.robotNode = self.rosNode.GetRobotNodeByName('smart_template')
    self.pubWorld = self.rosNode.GetPublisherNodeByTopic('/world_pose')
    self.pubGoal = self.rosNode.GetPublisherNodeByTopic('/desired_position')
    self.pubCommand = self.rosNode.GetPublisherNodeByTopic('/desired_command')
    self.pubEntry = self.rosNode.GetPublisherNodeByTopic('/planning/skin_entry')
    self.pubTarget = self.rosNode.GetPublisherNodeByTopic('/planning/target')
    self.subJoints = self.rosNode.GetSubscriberNodeByTopic('/joint_states')
    self.subPosition = self.rosNode.GetSubscriberNodeByTopic('/stage/state/guide_pose')
    if self.subJoints is not None:
      self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
    if self.subPosition is not None:
      self.observePosition = self.subPosition.AddObserver('ModifiedEvent', self.onSubPositionMsg)  

    self.link_names = None
    self.joint_names = None
    self.joint_limits = None
    self.desired_position = None

    self.mat_RobotToZFrame = None
    self.mat_ZFrameToScanner = None
    self.mat_RobotToScanner =  None
    self.mat_ScannerToRobot =  None
    self.eps = 0.45

    # If robot node is available, make sure the /robot_state_publisher is up
    if self.robotNode:
      paramNode = self.rosNode.GetParameterNodeByNode('/robot_state_publisher')
      if self.extractRobotParameters(paramNode) is False:
        print('Robot missing /robot_state_publisher')

    # MRML Objects for robot position
    # Robot to Scanner Transform node
    self.robotToScannerTransformNode = slicer.util.getFirstNodeByClassByName('vtkMRMLLinearTransformNode','RobotToScannerTransform')
    if self.robotToScannerTransformNode is None:
        self.robotToScannerTransformNode = slicer.vtkMRMLLinearTransformNode()
        self.robotToScannerTransformNode.SetName('RobotToScannerTransform')
        slicer.mrmlScene.AddNode(self.robotToScannerTransformNode)
    # Robot position message header
    self.robotPositionTimestamp = slicer.util.getFirstNodeByName('RobotPositionTimestamp', className='vtkMRMLTextNode')
    if self.robotPositionTimestamp is None:
      self.robotPositionTimestamp = slicer.vtkMRMLTextNode()
      self.robotPositionTimestamp.SetName('RobotPositionTimestamp')
      slicer.mrmlScene.AddNode(self.robotPositionTimestamp)
    # Robot position markups node
    self.robotPositionMarkupsNode= slicer.util.getFirstNodeByName('RobotPosition', className='vtkMRMLMarkupsFiducialNode')
    if self.robotPositionMarkupsNode is None:
      self.robotPositionMarkupsNode = slicer.vtkMRMLMarkupsFiducialNode()
      self.robotPositionMarkupsNode.SetName('RobotPosition')
      slicer.mrmlScene.AddNode(self.robotPositionMarkupsNode)
    self.initializeRobotPositionMarkup()

    # Create PointList node for planning
    self.pointListNode = slicer.util.getFirstNodeByName('Planning', className='vtkMRMLMarkupsFiducialNode')
    if self.pointListNode is None:
        self.pointListNode = slicer.vtkMRMLMarkupsFiducialNode()
        self.pointListNode.SetName('Planning')
        slicer.mrmlScene.AddNode(self.pointListNode)
    displayNodePointList = self.pointListNode.GetDisplayNode()
    if displayNodePointList:
      displayNodePointList.SetGlyphScale(1.5)   
    
    # Create ScriptedModule node for joint values
    self.jointValues = slicer.util.getFirstNodeByName('JointValues', className='vtkMRMLScriptedModuleNode')
    if self.jointValues is None:
      self.jointValues = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode', 'JointValues')

  ### Logic functions ###################################################################

  def initializeRobotPositionMarkup(self):
    # Ensure there is only one control point
    if self.robotPositionMarkupsNode.GetNumberOfControlPoints() > 1:
        self.robotPositionMarkupsNode.RemoveAllControlPoints()
    # If no control point exists, add one at (0,0,0)
    if self.robotPositionMarkupsNode.GetNumberOfControlPoints() == 0:
        self.robotPositionMarkupsNode.AddControlPoint(vtk.vtkVector3d(0, 0, 0), "")
    # Ensure tip is labeled, locked and one-point only
    self.robotPositionMarkupsNode.SetNthControlPointLabel(0, "")
    self.robotPositionMarkupsNode.SetLocked(True)
    self.robotPositionMarkupsNode.SetFixedNumberOfControlPoints(True)           
    displayNode = self.robotPositionMarkupsNode.GetDisplayNode()
    if displayNode:
        displayNode.SetGlyphScale(1.5)  # 1% glyph size and color
        displayNode.SetSelectedColor(0.667, 1.0, 0.498)
    else:
        # If the display node does not exist, create it and set the glyph size and color
        self.robotPositionMarkupsNode.CreateDefaultDisplayNodes()
        self.robotPositionMarkupsNode.GetDisplayNode().SetGlyphScale(1.5)
        displayNode.SetSelectedColor(0.667, 1.0, 0.498)
          # TODO: Set mat_RobotToScanner Transform 
    if self.robotToScannerTransformNode:
      self.robotPositionMarkupsNode.SetAndObserveTransformNodeID(self.robotToScannerTransformNode.GetID())

  # Initialize module parameter node with default settings
  def setDefaultParameters(self, parameterNode):
    # self.initialize()
    if not parameterNode.GetParameter('RobotLoaded'):
      if self.joint_names is None:
        isRobotLoaded = 'False'
      else:
        isRobotLoaded = 'True'
      parameterNode.SetParameter('RobotLoaded', isRobotLoaded)  
    if not parameterNode.GetNodeReference('Planning'):
      parameterNode.SetNodeReferenceID('Planning', self.pointListNode.GetID())

  # Print vtkMatrix4x4 (for debugging)
  def printVtkMatrix4x4(self, matrix4x4, name=''):
    print(name)
    for i in range(4):
      for j in range(4):
        print(matrix4x4.GetElement(i, j), end=" ")
      print()

  def formatTimestampToLocalTimeText(self, sec, nanosec):
    # Convert to seconds as float
    timestamp_sec = sec + nanosec / 1e9
    # Convert to local datetime
    dt = datetime.datetime.fromtimestamp(timestamp_sec)
    # Format: HH:MM:SS.mmm
    return dt.strftime('%H:%M:%S.') + f"{int(dt.microsecond / 1000):03d}"
  
  # Callback for /joints_state messages
  def onSubJointsMsg(self, caller=None, event=None):
    joints_msg = self.subJoints.GetLastMessage()
    # Extract joint names and positions from the message
    msg_joint_names = joints_msg.GetName()
    msg_joint_values = joints_msg.GetPosition()
    for joint_name, joint_value in zip(msg_joint_names, msg_joint_values):
      self.jointValues.SetAttribute(joint_name, str(1000*joint_value))  # Triggers observer
    self.jointValues.Modified()

  # Callback for /joints_state messages
  def onSubPositionMsg(self, caller=None, event=None):
    position_msg = self.subPosition.GetLastMessage()
    # Update timestamp string node
    stamp = position_msg.GetHeader().GetStamp()
    timestamp = self.formatTimestampToLocalTimeText(stamp.GetSec(), stamp.GetNanosec())
    self.robotPositionTimestamp.SetText(timestamp)
    self.robotPositionTimestamp.Modified()
    # Update position markups node
    point =  position_msg.GetPoint()
    position = [point.GetX(), point.GetY(), point.GetZ()]
    self.robotPositionMarkupsNode.SetNthControlPointPosition(0, position)
    self.robotPositionMarkupsNode.Modified()

  # Wait for robot models to be ready
  def onRobotModelReady(self, caller=None, event=None):
    N_model = self.robotNode.GetNumberOfNodeReferences('model')
    if(N_model >= len(self.link_names)):
      # print('Robot model %i available' %N_model)
      model_name = self.robotNode.GetNthNodeReferenceID('model', N_model-1)
      needle_node = slicer.mrmlScene.GetNodeByID(model_name)
      display_node = needle_node.GetDisplayNode()
      if display_node is None:
        needle_node.CreateDefaultDisplayNodes()
        display_node = needle_node.GetDisplayNode()
      # Change display settings
      display_node.SetVisibility2D(True)
      display_node.SetColor(1.0, 0.0, 1.0)
      display_node.SetSliceIntersectionThickness(2)
      caller.RemoveObserver(self.observeRobotModel)

  # Wait for robot_description parameter and then define link_names, joint_names and joint_limits  
  @vtk.calldata_type(vtk.VTK_OBJECT)
  def onRobotDescriptioReady(self, caller, calldata):
    if caller.GetMonitoredNodeName() == '/robot_state_publisher':
      print('Robot description available')
      if self.extractRobotParameters(caller):
        moduleParameterNode = self.getParameterNode()     # Update module parameter to help synch logic and gui
        moduleParameterNode.SetParameter('RobotLoaded', 'True')
        caller.RemoveObserver(self.observeRobotDescription)
        self.observeRobotModel = self.robotNode.AddObserver(slicer.vtkMRMLROS2RobotNode.ReferenceAddedEvent, self.onRobotModelReady)

  # Get last lookupNode if available    
  def getLastLookupNode(self):
    N_lookup = self.robotNode.GetNumberOfNodeReferences('lookup')
    if(N_lookup >= len(self.link_names)):
      lookup_name = self.robotNode.GetNthNodeReferenceID('lookup', N_lookup-1)
      return slicer.mrmlScene.GetNodeByID(lookup_name)
    else:
      return None

  # Get info from robot_description
  def extractRobotParameters(self, paramNode):
    if paramNode is None or not paramNode.IsParameterSet('robot_description'):
      print('Parameter node or robot description not available')
      return False
    robot_description = paramNode.GetParameterAsString('robot_description')
    print('Reading robot description...')
    try:
      root = ET.fromstring(robot_description)
    except ET.ParseError as e:
      print(f"[ERROR] URDF parsing failed: {e}")
      return False
    # Extract link names
    self.link_names = [link.get('name') for link in root.findall('link')]
    # Extract joint info with channel and limits
    joint_info = []
    for joint in root.findall('joint'):
      joint_name = joint.get('name')
      limit = joint.find('limit')
      channel_tag = joint.find('channel')
      if limit is None or channel_tag is None:
        continue  # Skip joints without necessary info
      try:
        channel = channel_tag.text.strip()
        lower = 1000 * float(limit.get('lower', '0.0'))
        upper = 1000 * float(limit.get('upper', '0.0'))
        joint_info.append((channel, joint_name, {'min': lower, 'max': upper}))
      except Exception as e:
        print(f"[WARNING] Skipping joint '{joint_name}' due to parsing error: {e}")
    if not joint_info:
      print("[ERROR] No valid joints with channel and limits found.")
      return False
    # Sort joints by channel
    joint_info.sort(key=lambda item: item[0])
    self.joint_names = [j[1] for j in joint_info]
    self.joint_limits = {j[1]: j[2] for j in joint_info}
    print(f"Robot links: {self.link_names}")
    print(f"Robot joints (sorted by channel): {self.joint_names}")
    # Extract ZFrame transform if available
    zframe_pose_element = root.find('./custom_parameters/zframe_pose')
    if zframe_pose_element is None:
      print('[ERROR] zframe_pose not found in custom_parameters.')
      return False
    try:
      zframe_pose = zframe_pose_element.get('value').strip()
      rows = zframe_pose.split('       ')  # split into rows
      matrix_values = [list(map(float, row.split())) for row in rows]
      for i in range(3):
        matrix_values[i][3] *= 1000  # scale translations
      mat_ZFrameToRobot = vtk.vtkMatrix4x4()
      for i in range(4):
        for j in range(4):
          mat_ZFrameToRobot.SetElement(i, j, matrix_values[i][j])
      self.mat_RobotToZFrame = vtk.vtkMatrix4x4()
      self.mat_RobotToZFrame.DeepCopy(mat_ZFrameToRobot)
      self.mat_RobotToZFrame.Invert()
      self.printVtkMatrix4x4(mat_ZFrameToRobot, 'ZFrameToRobot = ')
      return True
    except Exception as e:
      print(f"[ERROR] Failed to parse zframe_pose: {e}")
      return False
    
  def getJointValues(self):
    if self.joints is None:
      return None
    else:
      return self.joints

  def getDefinedJoints(self):
    if self.joint_names is None:
      return (None, None)
    else:
      return (self.joint_names, self.joint_limits)

  def loadRobot(self):
    # Create robot node and publisher to /world_pose message
    if self.robotNode is None:
      self.robotNode = self.rosNode.CreateAndAddRobotNode('smart_template','/robot_state_publisher','robot_description', 'world', '' ) 
      paramNode = slicer.util.getNode(self.robotNode.GetNodeReferenceID('parameter'))
      self.observeRobotDescription = paramNode.AddObserver(slicer.vtkMRMLROS2ParameterNode.ParameterModifiedEvent, self.onRobotDescriptioReady)
      if self.pubWorld is None:
          self.pubWorld = self.rosNode.CreateAndAddPublisherNode('TransformStamped', '/world_pose')
      if self.pubGoal is None:
        self.pubGoal = self.rosNode.CreateAndAddPublisherNode('Point', '/desired_position')
      if self.pubCommand is None:
        self.pubCommand = self.rosNode.CreateAndAddPublisherNode('String', '/desired_command')
      if self.pubEntry is None:
        self.pubEntry = self.rosNode.CreateAndAddPublisherNode('Point', '/planning/skin_entry')
      if self.pubTarget is None:
        self.pubTarget = self.rosNode.CreateAndAddPublisherNode('Point', '/planning/target')
      if self.subJoints is None:
        self.subJoints = self.rosNode.CreateAndAddSubscriberNode('JointState', '/joint_states')
        self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
      if self.subPosition is None:
        self.subPosition = self.rosNode.CreateAndAddSubscriberNode('PointStamped', '/stage/state/guide_pose')
        self.observePosition = self.subPosition.AddObserver('ModifiedEvent', self.onSubPositionMsg)  
      print('Robot initialized')
    else:
      # Should not enter here
      print('Robot already intialized')
    return True

  def registerRobot(self, ZFrameToScanner):
    if self.robotNode is None:
      print('Initialize robot first')
      return False
    if self.mat_RobotToScanner is None:
      self.mat_RobotToScanner =  vtk.vtkMatrix4x4()
      self.mat_ZFrameToScanner = vtk.vtkMatrix4x4()
      self.mat_ScannerToRobot =  vtk.vtkMatrix4x4()
    ZFrameToScanner.GetMatrixTransformToWorld(self.mat_ZFrameToScanner)
    vtk.vtkMatrix4x4.Multiply4x4(self.mat_ZFrameToScanner, self.mat_RobotToZFrame, self.mat_RobotToScanner)
    vtk.vtkMatrix4x4.Invert(self.mat_RobotToScanner, self.mat_ScannerToRobot)
    # Update robotToScanner transform
    self.robotToScannerTransformNode.SetMatrixTransformToParent(self.mat_RobotToScanner)   
    # Publish robotToScanner to world_pose broadcaster
    # TODO: Replace by a static broadcaster once it becomes available in SlicerROS2
    world_msg = self.pubWorld.GetBlankMessage()
    world_msg.SetTransform(self.mat_RobotToScanner)
    self.pubWorld.Publish(world_msg)
    self.printVtkMatrix4x4(self.mat_RobotToScanner, '\world_pose')
    return True
  
  def getRobotPosition(self, frame=None):
    if frame == 'world':
      return self.robotPositionMarkupsNode.GetNthControlPointPositionWorld(0)
    else:
      return self.robotPositionMarkupsNode.GetNthControlPointPosition(0)

  def getTimestamp(self):
    return self.robotPositionTimestamp.GetText()

  def getNumberOfPoints(self):
    if self.pointListNode is not None:
      return self.pointListNode.GetNumberOfDefinedControlPoints()
    else: 
      return 0
  
  def addPlanningPoint(self):
    if self.pointListNode is not None:
      N = self.pointListNode.GetNumberOfControlPoints()
      if N>=1:
        self.pointListNode.SetNthControlPointLabel(0, 'TARGET')
      if N>=2:
        self.pointListNode.SetNthControlPointLabel(1, 'ENTRY')
      if self.pointListNode.GetNumberOfDefinedControlPoints()>=2:
        self.pointListNode.SetFixedNumberOfControlPoints(2)

  # Place robot at the skin entry point
  def alignForInsertion(self):
    entry_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0] # Get entry point in scanner coordinates
    entry = list(self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)) # Calculate entry point in robot coordinates
    entry[1] = 0.0
    self.sendDesiredPosition(entry[:3])
    return True
    
  # Place robot at the skin entry point
  def approachSkin(self):
    # Get entry point in scanner coordinates
    entry_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    # Calculate entry point in robot coordinates
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    self.sendDesiredPosition(entry[:3])
    return True

  def isRobotAligned(self):
    entry_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    robot = [*self.robotPositionMarkupsNode.GetNthControlPointPosition(0), 1.0]
    if abs(robot[0] - entry[0]) < self.eps and abs(robot[2] - entry[2]) < self.eps:
      return True
    else:
      return False

  def isEntryAligned(self):
    # Get Points in scanner coordinates
    entry_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    target_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('TARGET')), 1.0]
    # Calculate Points in robot coordinates
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    # Align entry with target (in robot coordinates)
    aligned_entry = [target[0], entry[1], target[2], 1.0]
    if entry == aligned_entry:
      return True
    else:
      return False

  # Place robot at the skin entry point
  def insertStep(self, stepSize):
    goal = list(self.getRobotPosition())
    goal[1] += stepSize
    self.sendDesiredPosition(goal)
    return True
    
  def adjustEntry(self):
    # Get Points in scanner coordinates
    entry_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY')), 1.0]
    target_scanner = [*self.pointListNode.GetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('TARGET')), 1.0]
    # Calculate Points in robot coordinates
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    # Align entry with target (in robot coordinates)
    entry = [target[0], entry[1], target[2], 1.0]
    # Update entry scanner in markups list (to align with target)
    entry_scanner = self.mat_RobotToScanner.MultiplyPoint(entry)
    self.pointListNode.SetNthControlPointPosition(self.pointListNode.GetControlPointIndexByLabel('ENTRY'), entry_scanner[:3])
    return True

  def sendPlanning(self, entry, target):
    # Publish entry message
    entry_msg = self.pubEntry.GetBlankMessage()
    entry_msg.SetX(entry[0])
    entry_msg.SetY(entry[1])
    entry_msg.SetZ(entry[2])
    self.pubEntry.Publish(entry_msg)
    print('Sent entry point: %s' %(entry))
    # Publish target message
    target_msg = self.pubTarget.GetBlankMessage()
    target_msg.SetX(target[0])
    target_msg.SetY(target[1])
    target_msg.SetZ(target[2])
    self.pubTarget.Publish(target_msg)
    print('Sent target: %s' %(target))

  def sendDesiredPosition(self, goal):
    # Publish desired_position message
    self.desired_position = goal
    goal_msg = self.pubGoal.GetBlankMessage()
    goal_msg.SetX(self.desired_position[0])
    goal_msg.SetY(self.desired_position[1])
    goal_msg.SetZ(self.desired_position[2])
    self.pubGoal.Publish(goal_msg)
    print(self.desired_position)

  def sendDesiredCommand(self, command):
    # Publish desired_command message
    self.pubCommand.Publish(command)
