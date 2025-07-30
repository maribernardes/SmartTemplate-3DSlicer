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
import time

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
    robotCollapsibleButton.text = 'SmartTemplate Robot'
    self.layout.addWidget(robotCollapsibleButton)

    # Load robot button
    robotFormLayout = qt.QFormLayout(robotCollapsibleButton)
    
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
    robotFormLayout.addRow(registrationLayout)

    # Load, Register and Correct Calibration buttons
    initButtonLayout = qt.QHBoxLayout()
    self.loadButton = qt.QPushButton('Load')
    self.loadButton.setFixedWidth(150)
    self.loadButton.toolTip = 'Loads SmartTemplate robot in 3DSlicer'
    self.loadButton.enabled = True    
    self.registerButton = qt.QPushButton('Register')
    self.registerButton.setFixedWidth(150)
    self.registerButton.toolTip = 'Registers SmartTemplate robot to scanner'
    self.registerButton.enabled = False
    self.correctCalibrationButton = qt.QPushButton('Correct calibration')
    self.correctCalibrationButton.setFixedWidth(150)
    self.correctCalibrationButton.toolTip = 'Correct robot insertion joint calibration'
    self.correctCalibrationButton.enabled = False
    initButtonLayout.addStretch()
    initButtonLayout.addWidget(self.loadButton)
    initButtonLayout.addWidget(self.registerButton)
    initButtonLayout.addWidget(self.correctCalibrationButton)
    initButtonLayout.addStretch()
    robotFormLayout.addRow(initButtonLayout)
    robotFormLayout.addRow('', qt.QLabel(''))  # Vertical space

    # Robot joints    
    jointsFrame = qt.QFrame()
    jointsFrame.setFrameShape(qt.QFrame.Box)            # Draw a rectangular border
    jointsFrame.setFrameShadow(qt.QFrame.Sunken)        # Optional: sunken look
    jointsFrame.setLineWidth(1)
    jointsFrame.setMinimumHeight(200)
    self.jointsLayout = qt.QVBoxLayout(jointsFrame)
    robotFormLayout.addRow(jointsFrame)

    # Robot position
    positionLayout = qt.QHBoxLayout()
    positionRASLabel = qt.QLabel('RAS:')
    self.positionRASTextbox = qt.QLineEdit('( ---.--, ---.--, ---.-- )')
    self.positionRASTextbox.setReadOnly(True)
    self.positionRASTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRASTextbox.toolTip = 'Robot current position (scanner frame)'
    self.positionRASTextbox.setFixedWidth(165)
    separationLabel = qt.QLabel(' / ')
    positionLabel = qt.QLabel('XYZ:')
    self.positionRobotTextbox = qt.QLineEdit('( ---.--, ---.--, ---.-- )')
    self.positionRobotTextbox.setReadOnly(True)
    self.positionRobotTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.positionRobotTextbox.toolTip = 'Robot current position (robot frame)'
    self.positionRobotTextbox.setFixedWidth(165)
    self.timeStampTextbox = qt.QLineEdit('--:--:--.----')
    self.timeStampTextbox.setReadOnly(True)
    self.timeStampTextbox.setStyleSheet('background-color: transparent; border: no border;')
    self.timeStampTextbox.toolTip = 'Timestamp from last robot position'
    
    positionLayout.addWidget(positionRASLabel)
    positionLayout.addWidget(self.positionRASTextbox)
    positionLayout.addWidget(separationLabel)
    positionLayout.addWidget(positionLabel)
    positionLayout.addWidget(self.positionRobotTextbox)
    positionLayout.addStretch()
    positionLayout.addWidget(self.timeStampTextbox)
    robotFormLayout.addRow(positionLayout)

    robotButtonsLayout = qt.QHBoxLayout() 
    self.homeButton = qt.QPushButton('HOME')
    self.homeButton.setFixedWidth(150)
    self.homeButton.toolTip = 'Go to robot HOME position'
    self.homeButton.enabled = False
    self.retractButton = qt.QPushButton('RETRACT')
    self.retractButton.setFixedWidth(150)
    self.retractButton.toolTip = 'Fully retract needle'
    self.retractButton.enabled = False

    robotButtonsLayout.addStretch()
    robotButtonsLayout.addWidget(self.homeButton)
    robotButtonsLayout.addWidget(self.retractButton)
    robotButtonsLayout.addStretch()
    robotFormLayout.addRow('', qt.QLabel(''))  # Vertical space
    robotFormLayout.addRow(robotButtonsLayout)
    robotFormLayout.addRow('', qt.QLabel(''))  # Vertical space

    ## Planning collapsible button                 
    ####################################
    
    planningCollapsibleButton = ctk.ctkCollapsibleButton()
    planningCollapsibleButton.text = 'Planning'
    self.layout.addWidget(planningCollapsibleButton)   
    planningFormLayout = qt.QVBoxLayout(planningCollapsibleButton)
    
    # Points selection
    markupsLayout = qt.QFormLayout()
    planningFormLayout.addLayout(markupsLayout)
    self.planningSelector = slicer.qSlicerSimpleMarkupsWidget()    
    self.planningSelector.setMRMLScene(slicer.mrmlScene)
    self.planningSelector.setNodeSelectorVisible(True)
    self.planningSelector.markupsPlaceWidget().setPlaceMultipleMarkups(True)
    self.planningSelector.defaultNodeColor = qt.QColor(170,0,0)
    self.planningSelector.setMaximumHeight(120)
    self.planningSelector.tableWidget().show()
    self.planningSelector.toolTip = 'Select 2 points: TARGET and ENTRY'
    self.planningSelector.setEnterPlaceModeOnNodeChange(False)
    markupsLayout.addRow('Planned points:', self.planningSelector)
    
    # Adjust Entry button 
    planningButtonsLayout = qt.QHBoxLayout()   
    self.adjustEntryButton = qt.QPushButton('Adjust ENTRY to TARGET')
    self.adjustEntryButton.toolTip = 'Adjust ENTRY to have it aligned to TARGET'
    self.adjustEntryButton.enabled = False
    self.publishPlanningButton = qt.QPushButton('Publish Planning')
    self.publishPlanningButton.toolTip = 'Publish ENTRY and TARGET to ROS2'
    self.publishPlanningButton.enabled = False
    planningButtonsLayout.addWidget(self.adjustEntryButton)
    planningButtonsLayout.addWidget(self.publishPlanningButton)
    markupsLayout.addRow('', planningButtonsLayout)
    markupsLayout.addRow('', qt.QLabel(''))  # Vertical space

    commandButtonsLayout = qt.QHBoxLayout()    
    self.alignButton = qt.QPushButton('ALIGN')
    self.alignButton.setFixedWidth(150)
    self.alignButton.toolTip = 'Align robot to entry'
    self.alignButton.enabled = False
    self.approachButton = qt.QPushButton('APPROACH')
    self.approachButton.setFixedWidth(150)
    self.approachButton.toolTip = 'Approach robot to skin'
    self.approachButton.enabled = False

    commandButtonsLayout.addStretch()
    commandButtonsLayout.addWidget(self.alignButton)
    commandButtonsLayout.addWidget(self.approachButton)
    commandButtonsLayout.addStretch()
    markupsLayout.addRow(commandButtonsLayout)

## Insertion collapsible button                
    ####################################
    
    insertionCollapsibleButton = ctk.ctkCollapsibleButton()
    insertionCollapsibleButton.text = 'Insertion'    
    self.layout.addWidget(insertionCollapsibleButton)

    insertionFormLayout = qt.QFormLayout(insertionCollapsibleButton)

    # Track needle tip using OpenIGTLink connection
    trackTipLayout = qt.QHBoxLayout()
    self.igtlConnectionSelector = slicer.qMRMLNodeComboBox()
    self.igtlConnectionSelector.nodeTypes = ['vtkMRMLIGTLConnectorNode']
    self.igtlConnectionSelector.selectNodeUponCreation = True
    self.igtlConnectionSelector.addEnabled = False
    self.igtlConnectionSelector.removeEnabled = False
    self.igtlConnectionSelector.noneEnabled = True
    self.igtlConnectionSelector.showHidden = False
    self.igtlConnectionSelector.showChildNodeTypes = False
    self.igtlConnectionSelector.setMRMLScene(slicer.mrmlScene)
    self.igtlConnectionSelector.setToolTip('Select OpenIGTLink server')
    self.igtlConnectionSelector.enabled = True
    self.igtlConnectionSelector.setSizePolicy(qt.QSizePolicy.Expanding, qt.QSizePolicy.Preferred)
    trackTipLayout.addWidget(qt.QLabel('IGTLServer Tip:'))
    trackTipLayout.addWidget(self.igtlConnectionSelector, 1)
    self.recordInsertionButton = qt.QPushButton("Record Insertion")
    self.recordInsertionButton.toolTip = "Record tip positions during insertion"
    self.recordInsertionButton.enabled = False
    trackTipLayout.addWidget(self.recordInsertionButton)
    insertionFormLayout.addRow(trackTipLayout)
    insertionFormLayout.addRow('', qt.QLabel(''))  # Vertical space
    
    insertionButtonsLayout = qt.QHBoxLayout()
    self.toTargetButton= qt.QPushButton('Insert to target')
    self.toTargetButton.setFixedWidth(150)
    self.toTargetButton.toolTip = 'Insert the needle to target'
    self.toTargetButton.enabled = False

    self.stepButton= qt.QPushButton('Insert or Retract by =>')
    self.stepButton.setFixedWidth(150)    
    self.stepButton.toolTip = 'Insert the needle stepwise a certain lenght'
    self.stepButton.enabled = False

    self.stepSizeTextbox = qt.QLineEdit('20.0')
    self.stepSizeTextbox.setReadOnly(False)
    self.stepSizeTextbox.setMaximumWidth(50)
    insertionButtonsLayout.addStretch()
    insertionButtonsLayout.addWidget(self.toTargetButton)
    insertionButtonsLayout.addWidget(self.stepButton)
    insertionButtonsLayout.addWidget(self.stepSizeTextbox)
    insertionButtonsLayout.addWidget(qt.QLabel('mm'))
    insertionButtonsLayout.addStretch()
    insertionFormLayout.addRow(insertionButtonsLayout)

    self.layout.addStretch(1)
    
    ####################################
    ##                                ##
    ## UI Behavior                    ##
    ##                                ##
    ####################################

    # Internal variables
    self.isRobotLoaded = False  # Is the robot loaded?
    self.isTipTracked = False   # Is the needle tip tracked?
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
    self.addObserver(self.logic.robotPositionNode, vtk.vtkCommand.ModifiedEvent, self.onPositionChange)
    self.addObserver(self.logic.trackedTipNode, slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onTrackedTipChange)

    # These connections ensure that whenever user changes some settings on the GUI, that is saved in the MRML scene
    # (in the selected parameter node).
    self.zTransformSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.updateParameterNodeFromGUI)
    self.planningSelector.connect('markupsFiducialNodeChanged()', self.updateParameterNodeFromGUI)
    self.igtlConnectionSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.updateParameterNodeFromGUI)
    
    # Connect Qt widgets to event calls
    self.loadButton.connect('clicked(bool)', self.loadRobot)
    self.registerButton.connect('clicked(bool)', self.registerRobot)
    self.correctCalibrationButton.connect('clicked(bool)', self.correctCalibration)
    self.adjustEntryButton.connect('clicked(bool)', self.adjustEntry)
    self.publishPlanningButton.connect('clicked(bool)', self.publishPlanning)
    self.retractButton.connect('clicked(bool)', self.retractNeedle)
    self.homeButton.connect('clicked(bool)', self.homeRobot)
    self.alignButton.connect('clicked(bool)', self.alignForInsertion)
    self.approachButton.connect('clicked(bool)', self.approachSkin)
    self.toTargetButton.connect('clicked(bool)', self.toTarget)
    self.stepButton.connect('clicked(bool)', self.insertStep)
    self.planningSelector.connect('markupsFiducialNodeChanged()', self.onSelectMarkups)
    self.planningSelector.connect('updateFinished()', self.onPlanningChanged)
    self.recordInsertionButton.connect('clicked(bool)', self.onRecordInsertion)

    
    # Initialize widget variables and updateGUI
    self.updateGUI()

  ### Widget functions ###################################################################
  # Called when the application closes and the module widget is destroyed.
  def cleanup(self):
    self.removeObservers()

  # Called each time the user opens this module.
  # Make sure parameter node exists and observed
  def enter(self):
    self.initializeParameterNode()
    self.logic.initializeInternalNodes() 

  # Called each time the user opens a different module.
  # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
  def exit(self):
    self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self.updateGUIFromParameterNode)

  # Called just before the scene is closed.
  # Parameter node will be reset, do not use it anymore
  def onSceneStartClose(self, caller, event):
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
    self.isRobotLoaded = self._parameterNode.GetParameter('RobotLoaded') == 'True'
    self.isTipTracked = self._parameterNode.GetParameter('TipTracked') == 'True'
    self.zTransformSelector.setCurrentNode(self._parameterNode.GetNodeReference('ZTransform'))
    self.planningSelector.setCurrentNode(self._parameterNode.GetNodeReference('Planning'))
    self.igtlConnectionSelector.setCurrentNode(self._parameterNode.GetNodeReference('TipIGTLServer'))
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
    planningMarkupsNode = self.planningSelector.currentNode()
    self._parameterNode.SetNodeReferenceID('Planning', planningMarkupsNode.GetID() if planningMarkupsNode else None)
    self._parameterNode.SetNodeReferenceID('TipIGTLServer', self.igtlConnectionSelector.currentNodeID)
    # All paramenter_nodes updates are done
    self._parameterNode.EndModify(wasModified)

  # Called when a MarkupsNode is selected
  def onSelectMarkups(self):
    planningMarkupsNode = self.planningSelector.currentNode()
    if planningMarkupsNode:
      planningMarkupsNode.SetRequiredNumberOfControlPoints(2)
      displayNode = planningMarkupsNode.GetDisplayNode()
      if displayNode:
        displayNode.SetGlyphScale(1.5)   

  # Called when the Point List is updated
  def onPlanningChanged(self):
    self.logic.addPlanningPoint(self.planningSelector.currentNode())
    self.updateGUI()

  # Update GUI buttons, markupWidget and connection statusLabel
  def updateGUI(self):
    # Check status
    robotRegistered = False if self.logic.mat_RobotToScanner is None else True
    zFrameSelected = (self.zTransformSelector.currentNode() is not None)
    planningMarkupsNode = self.planningSelector.currentNode()
    pointsSelected = (self.logic.getNumberOfPoints(planningMarkupsNode) == 2)
    if (pointsSelected and zFrameSelected and robotRegistered):
      robotAligned = self.logic.isRobotAligned(planningMarkupsNode)
    else:
      robotAligned = False
    
    # Update buttons accordingly
    self.loadButton.enabled = not self.isRobotLoaded
    self.registerButton.enabled = zFrameSelected and self.isRobotLoaded
    self.correctCalibrationButton.enabled = robotRegistered and self.isRobotLoaded and self.isTipTracked
    self.adjustEntryButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.publishPlanningButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.homeButton.enabled = self.isRobotLoaded
    self.retractButton.enabled = self.isRobotLoaded
    self.stepButton.enabled = self.isRobotLoaded
    self.toTargetButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.alignButton.enabled = pointsSelected and robotRegistered and self.isRobotLoaded
    self.approachButton.enabled = robotAligned and pointsSelected and robotRegistered and self.isRobotLoaded
    self.recordInsertionButton.enabled = (self.igtlConnectionSelector.currentNode() is not None)

    # Add joints if not already added:
    if self.jointNames is None and self.isRobotLoaded:
      self.addJoints()

  def format3DPoint(self, point, frame=None, decimals=2):
    def format_coord(value):
      # Format: space for sign + 3 digits + dot + decimals
      # Total width = sign + 3 + 1 + decimals → e.g. for 2 decimals: width = 7
      return f"{value:+7.{decimals}f}".replace("+", " ")

    formatted = f"({format_coord(point[0])}, {format_coord(point[1])}, {format_coord(point[2])} )"
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
      for i, joint in enumerate(self.jointNames):
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
        # Add a horizontal separator line between joints
        if i < len(self.jointNames) - 1:
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
    robotPositionXYZ, robotTimeStamp = self.logic.getRobotPosition()
    #print('Time: %s / Tip (RAS): %s' %(timestamp,tipCoordinates))
    if robotPositionXYZ is not None:
      self.positionRobotTextbox.setText(self.format3DPoint(robotPositionXYZ))
    else:
      self.positionRobotTextbox.setText('(---, ---, ---)')
    robotPositionRAS, _ = self.logic.getRobotPosition('world')
    if robotPositionRAS is not None:
      self.positionRASTextbox.setText(self.format3DPoint(robotPositionRAS))
    else:
      self.positionRASTextbox.setText('(---, ---, ---)')
    self.timeStampTextbox.setText(robotTimeStamp)
    self.updateGUI()

  # Synch experiment data when new tracked tip is received
  def onTrackedTipChange(self, caller=None, event=None):
    self.logic.updateTrackedTip()

  def onRecordInsertion(self):
    if not self.logic.loggingActive:
        self.logic.startLogging()
        self.recordInsertionButton.setText("Stop && Save Insertion")
    else:
        self.logic.stopAndSaveLogging(self.planningSelector.currentNode())
        self.recordInsertionButton.setText("Record Insertion")


  def loadRobot(self):
    print('UI: loadRobot()')
    self.logic.loadRobot()
    print('____________________')
    self.updateGUI()

  def registerRobot(self):
    print('UI: registerRobot()')
    zTransformNode = self.zTransformSelector.currentNode()
    self.logic.registerRobot(zTransformNode)
    print('____________________')
    self.updateGUI()

  def correctCalibration(self):
    print('UI: correctCalibration()')
    zTransformNode = self.zTransformSelector.currentNode()
    newZTransformNode = self.logic.correctCalibration(zTransformNode)
    self.zTransformSelector.setCurrentNode(newZTransformNode)
    print('____________________')
    print(newZTransformNode.GetName())
    self.updateGUI()

  def adjustEntry(self):
    print('UI: adjustEntry()')
    self.logic.adjustEntry(self.planningSelector.currentNode())
    print('____________________')
    self.updateGUI()

  def publishPlanning(self):
    print('UI: publishPlanning()')
    self.logic.publishPlanning(self.planningSelector.currentNode())
    print('____________________')
    self.updateGUI()

  def retractNeedle(self):
    print('UI: retractNeedle()')
    self.logic.sendDesiredCommand('RETRACT')
    print('____________________')
    self.updateGUI()

  def homeRobot(self):
    print('UI: homeRobot()')
    self.logic.sendDesiredCommand('HOME')
    print('____________________')
    self.updateGUI()

  def alignForInsertion(self):
    print('UI: alignForInsertion()')
    self.logic.alignForInsertion(self.planningSelector.currentNode())
    print('____________________')
    self.updateGUI()

  def approachSkin(self):
    print('UI: approachSkin()')
    self.logic.approachSkin(self.planningSelector.currentNode())
    print('____________________')
    self.updateGUI()

  def toTarget(self):
    print('UI: toTarget()')
    self.logic.toTarget(self.planningSelector.currentNode())
    print('____________________')
    self.updateGUI()

  def insertStep(self):
    print('UI: insertStep()')
    self.logic.insertStep(float(self.stepSizeTextbox.text.strip()))
    print('____________________')
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
    self.pubEntry = self.rosNode.GetPublisherNodeByTopic('/planned_entry')
    self.pubTarget = self.rosNode.GetPublisherNodeByTopic('/planned_target')
    self.pubTrackedTip = self.rosNode.GetPublisherNodeByTopic('/tracked_tip')
    self.subJoints = self.rosNode.GetSubscriberNodeByTopic('/joint_states')
    self.lookupPosition = self.rosNode.GetTf2LookupNodeByParentChild('world','needle_link')
    if self.subJoints is not None:
      self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
    if self.lookupPosition is not None:
      print('loaded existing lookup')
      self.observePosition = self.lookupPosition.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onPositionUpdate)

    self.link_names = None
    self.joint_names = None
    self.joint_limits = None
    self.desired_position = None

    self.mat_ZFrameToRobot = None
    self.mat_RobotToZFrame = None
    self.mat_ZFrameToScanner = None
    self.mat_ScannerToZFrame = None
    self.mat_RobotToScanner =  None
    self.mat_ScannerToRobot =  None

    self.loggingActive = False
    self.loggedRobotPositions = []    # Stored robot position (RAS) at tip receipt time
    self.loggedRobotTimestamps = []   # When robot position was stored
    self.loggedTipPositions = []      # Tip position
    self.loggedTipTimestamps = []     # From tip message header (other computer timestamp)
    self.loggedLogTimestamps = []     # When tip was received 

    self.eps = 0.6

    # If robot node is available, make sure the /robot_state_publisher is up
    if self.robotNode:
      paramNode = self.rosNode.GetParameterNodeByNode('/robot_state_publisher')
      if self.extractRobotParameters(paramNode) is False:
        print('Robot missing /robot_state_publisher')

    # MRML Objects for robot position
    self.initializeInternalNodes()
    self.initializeRobotPosition()
    self.initializeTrackedTip()


  ### Logic functions ###################################################################

  # Initialize module parameter node with default settings
  def setDefaultParameters(self, parameterNode):
    # self.initialize()
    if not parameterNode.GetParameter('RobotLoaded'):
      if self.joint_names is None:
        isRobotLoaded = 'False'
      else:
        isRobotLoaded = 'True'
      parameterNode.SetParameter('RobotLoaded', isRobotLoaded)  
    if not parameterNode.GetParameter('TipTracked'):
      parameterNode.SetParameter('TipTracked', 'False')  

  def initializeInternalNodes(self):
    # Robot to Scanner Transform node
    self.robotToScannerTransformNode = slicer.util.getFirstNodeByClassByName('vtkMRMLLinearTransformNode','RobotToScannerTransform')
    if self.robotToScannerTransformNode is None:
        self.robotToScannerTransformNode = slicer.vtkMRMLLinearTransformNode()
        self.robotToScannerTransformNode.SetName('RobotToScannerTransform')
        self.robotToScannerTransformNode.SetHideFromEditors(True)
        slicer.mrmlScene.AddNode(self.robotToScannerTransformNode)
    # Robot position message header
    self.robotPositionTimestampNode = slicer.util.getFirstNodeByName('RobotPositionTimestamp', className='vtkMRMLTextNode')
    if self.robotPositionTimestampNode is None:
      self.robotPositionTimestampNode = slicer.vtkMRMLTextNode()
      self.robotPositionTimestampNode.SetName('RobotPositionTimestamp')
      slicer.mrmlScene.AddNode(self.robotPositionTimestampNode)
    self.robotPositionNode = slicer.util.getFirstNodeByClassByName('vtkMRMLLinearTransformNode','RobotPositionTransform')
    if self.robotPositionNode is None:
        self.robotPositionNode = slicer.vtkMRMLLinearTransformNode()
        self.robotPositionNode.SetName('RobotPositionTransform')
        self.robotPositionNode.SetHideFromEditors(True)
        slicer.mrmlScene.AddNode(self.robotPositionNode)
    self.robotPositionMarkupsNode= slicer.util.getFirstNodeByName('RobotPosition', className='vtkMRMLMarkupsFiducialNode')
    if self.robotPositionMarkupsNode is None:
      self.robotPositionMarkupsNode = slicer.vtkMRMLMarkupsFiducialNode()
      self.robotPositionMarkupsNode.SetName('RobotPosition')
      slicer.mrmlScene.AddNode(self.robotPositionMarkupsNode)
    # Tracked tip (optional)
    self.needleConfidenceNode = slicer.util.getFirstNodeByClassByName('vtkMRMLTextNode','CurrentTipConfidence')
    if self.needleConfidenceNode is None:
        self.needleConfidenceNode = slicer.vtkMRMLTextNode()
        self.needleConfidenceNode.SetName('CurrentTipConfidence')
        slicer.mrmlScene.AddNode(self.needleConfidenceNode)
    self.trackedTipNode = slicer.util.getFirstNodeByClassByName('vtkMRMLLinearTransformNode','CurrentTrackedTip')
    if self.trackedTipNode is None:
        self.trackedTipNode = slicer.vtkMRMLLinearTransformNode()
        self.trackedTipNode.SetName('CurrentTrackedTip')
        slicer.mrmlScene.AddNode(self.trackedTipNode)
    self.trackedTipMarkupsNode= slicer.util.getFirstNodeByName('TrackedTip', className='vtkMRMLMarkupsFiducialNode')
    if self.trackedTipMarkupsNode is None:
      self.trackedTipMarkupsNode = slicer.vtkMRMLMarkupsFiducialNode()
      self.trackedTipMarkupsNode.SetName('TrackedTip')
      slicer.mrmlScene.AddNode(self.trackedTipMarkupsNode)

    # Create ScriptedModule node for joint values
    self.jointValues = slicer.util.getFirstNodeByName('JointValues', className='vtkMRMLScriptedModuleNode')
    if self.jointValues is None:
      self.jointValues = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLScriptedModuleNode', 'JointValues')

  def initializeRobotPosition(self):
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
        displayNode.SetVisibility(False)
        displayNode.SetSelectedColor(1.0, 1.0, 1.0)
    else:
        # If the display node does not exist, create it and set the glyph size and color
        self.robotPositionMarkupsNode.CreateDefaultDisplayNodes()
        self.robotPositionMarkupsNode.GetDisplayNode().SetGlyphScale(1.5)
        displayNode.SetVisibility(False)
        displayNode.SetSelectedColor(1.0, 1.0, 1.0)
    self.robotPositionMarkupsNode.SetAndObserveTransformNodeID(self.robotPositionNode.GetID())
    self.robotPositionNode.SetAndObserveTransformNodeID(self.robotToScannerTransformNode.GetID())

  def initializeTrackedTip(self):
    # Ensure there is only one control point
    if self.trackedTipMarkupsNode.GetNumberOfControlPoints() > 1:
        self.trackedTipMarkupsNode.RemoveAllControlPoints()
    # If no control point exists, add one at (0,0,0)
    if self.trackedTipMarkupsNode.GetNumberOfControlPoints() == 0:
        self.trackedTipMarkupsNode.AddControlPoint(vtk.vtkVector3d(0, 0, 0), "")
    # Ensure tip is labeled, locked and one-point only
    self.trackedTipMarkupsNode.SetNthControlPointLabel(0, "")
    self.trackedTipMarkupsNode.SetLocked(True)
    self.trackedTipMarkupsNode.SetFixedNumberOfControlPoints(True)           
    displayNode = self.trackedTipMarkupsNode.GetDisplayNode()
    if displayNode:
        displayNode.SetGlyphScale(1.5)  # 1% glyph size and color
        self.setTipMarkupColor(None)
    else:
        # If the display node does not exist, create it and set the glyph size and color
        self.trackedTipMarkupsNode.CreateDefaultDisplayNodes()
        self.trackedTipMarkupsNode.GetDisplayNode().SetGlyphScale(1.5)
        self.setTipMarkupColor(None)
    self.trackedTipMarkupsNode.SetAndObserveTransformNodeID(self.trackedTipNode.GetID())
    self.getParameterNode().SetParameter('TipTracked', 'False')

  # Set TipMarkup Color according to tracking status
  def setTipMarkupColor(self, tipTracked:bool = None):
    displayNode = self.trackedTipMarkupsNode.GetDisplayNode()
    if tipTracked is None:
      displayNode.SetVisibility(False)
      displayNode.SetSelectedColor(1.0, 0.502, 0.502) #PINK (default)
    elif tipTracked is True:
      displayNode.SetVisibility(True)
      displayNode.SetSelectedColor(0.667, 1.0, 0.498) #GREEN (default)
    else:
      displayNode.SetVisibility(True)
      displayNode.SetSelectedColor(1.0, 1.0, 0.498) #YELLOW (default)

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

  # Callback for lookup update
  def onPositionUpdate(self, caller=None, event=None):
    # Update timestamp string node
    timestamp = time.time()
    dt = datetime.datetime.fromtimestamp(timestamp)
    logTimestamp = dt.strftime('%H:%M:%S.%f')[:-3]
    self.robotPositionTimestampNode.SetText(logTimestamp)
    self.robotPositionTimestampNode.Modified()
    # Update the position value
    robotPositionMatrix = vtk.vtkMatrix4x4()
    self.lookupPosition.GetMatrixTransformToParent(robotPositionMatrix)
    self.robotPositionNode.SetMatrixTransformToParent(robotPositionMatrix)
    self.robotPositionNode.Modified()

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
    print(robot_description)
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
      self.mat_ZFrameToRobot = vtk.vtkMatrix4x4()
      for i in range(4):
        for j in range(4):
          self.mat_ZFrameToRobot.SetElement(i, j, matrix_values[i][j])
      self.mat_RobotToZFrame = vtk.vtkMatrix4x4()
      vtk.vtkMatrix4x4.Invert(self.mat_ZFrameToRobot, self.mat_RobotToZFrame)
      self.printVtkMatrix4x4(self.mat_ZFrameToRobot, 'ZFrameToRobot = ')

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
        self.pubEntry = self.rosNode.CreateAndAddPublisherNode('Point', '/planned_entry')
      if self.pubTarget is None:
        self.pubTarget = self.rosNode.CreateAndAddPublisherNode('Point', '/planned_target')
      if self.pubTrackedTip is None:
        self.pubTrackedTip = self.rosNode.CreateAndAddPublisherNode('Point', '/tracked_tip')
      if self.subJoints is None:
        self.subJoints = self.rosNode.CreateAndAddSubscriberNode('JointState', '/joint_states')
        self.observeJoints = self.subJoints.AddObserver('ModifiedEvent', self.onSubJointsMsg)
      if self.lookupPosition is None:
        print('create lookup')
        self.lookupPosition = self.rosNode.CreateAndAddTf2LookupNode('world','needle_link')
        self.observePosition = self.lookupPosition.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.onPositionUpdate)
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
      self.mat_ScannerToRobot =  vtk.vtkMatrix4x4()
      self.mat_ZFrameToScanner = vtk.vtkMatrix4x4()
      self.mat_ScannerToZFrame = vtk.vtkMatrix4x4()
      
    ZFrameToScanner.GetMatrixTransformToWorld(self.mat_ZFrameToScanner)
    vtk.vtkMatrix4x4.Invert(self.mat_ZFrameToScanner, self.mat_ScannerToZFrame)    
    vtk.vtkMatrix4x4.Multiply4x4(self.mat_ZFrameToScanner, self.mat_RobotToZFrame, self.mat_RobotToScanner)
    vtk.vtkMatrix4x4.Invert(self.mat_RobotToScanner, self.mat_ScannerToRobot)
    self.robotToScannerTransformNode.SetMatrixTransformToParent(self.mat_RobotToScanner)  # Update robotToScanner transform

    # Publish robotToScanner to world_pose broadcaster
    # TODO: Replace by a static broadcaster once it becomes available in SlicerROS2
    world_msg = self.pubWorld.GetBlankMessage()
    world_msg.SetTransform(self.mat_RobotToScanner)
    self.pubWorld.Publish(world_msg)
    self.printVtkMatrix4x4(self.mat_RobotToScanner, '\world_pose')
    # Update robot position markups node
    displayNode = self.robotPositionMarkupsNode.GetDisplayNode()
    displayNode.SetVisibility(True)
    return True
  
  # Correct calibration error
  def correctCalibration(self, ZFrameToScanner):
    if self.robotNode is None:
      print('Initialize robot first')
      return None
    elif self.mat_RobotToScanner is None:
      print('Register robot first')
      return None
    # Calculate error in depth
    robotPosition, _ = self.getRobotPosition('world')
    tipPosition = self.getTipPosition()
    insertion_axis_robot = [0.0, 1.0, 0.0, 0.0]     # Insertion axis in robot frame (Y axis) homogeneous vector (w=0)
    insertion_axis_scanner = [0.0, 0.0, 0.0, 0.0]   # Initialize insertion axis in scanner frame
    self.mat_RobotToScanner.MultiplyPoint(insertion_axis_robot, insertion_axis_scanner) # Calculate insertion axis in scanner frame
    axis = insertion_axis_scanner[:3] # Remove w component
    vtk.vtkMath.Normalize(axis)       # Normalize axis vector
    diff = [0.0, 0.0, 0.0]
    vtk.vtkMath.Subtract(tipPosition, robotPosition, diff) # Calculate difference
    insertion_error = vtk.vtkMath.Dot(diff, axis) # Project onto insertion axis
    # Compute offset = insertion_error * axis
    offset = [component * insertion_error for component in axis]
    originalMatrix = vtk.vtkMatrix4x4()
    ZFrameToScanner.GetMatrixTransformToParent(originalMatrix)
    # Create copy of the original registration node
    newZFrameToScanner = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode")
    newName = self.getUniqueNodeName(ZFrameToScanner.GetName() + "_CORRECTED")
    newZFrameToScanner.SetName(newName)
    newZFrameToScanner.SetMatrixTransformToParent(originalMatrix)
    print(f"Created copy of '{ZFrameToScanner.GetName()}' as '{newName}'")
    # Update translation part of ZFrameToScanner
    updatedMatrix = vtk.vtkMatrix4x4()
    updatedMatrix.DeepCopy(originalMatrix)
    for i in range(3):
        updatedMatrix.SetElement(i, 3, originalMatrix.GetElement(i, 3) + offset[i])
    newZFrameToScanner.SetMatrixTransformToParent(updatedMatrix)
    newZFrameToScanner.Modified()
    print(f"Updated registration '{newZFrameToScanner.GetName()}' by {insertion_error:.2f} mm along insertion axis.")
    self.registerRobot(newZFrameToScanner)
    return newZFrameToScanner

  def getTranslation(self, linearTransformNode, frame=None):
    matrix = vtk.vtkMatrix4x4()
    if frame == 'world':
      linearTransformNode.GetMatrixTransformToWorld(matrix)
    else:
      linearTransformNode.GetMatrixTransformToParent(matrix)
    # Extract translation (last column of matrix, ignoring bottom row)
    tx = matrix.GetElement(0, 3)
    ty = matrix.GetElement(1, 3)
    tz = matrix.GetElement(2, 3)
    return [tx, ty, tz]

  def getRobotPosition(self, frame=None):
    return (self.getTranslation(self.robotPositionNode, frame=frame), self.robotPositionTimestampNode.GetText())

  def getTipPosition(self):
    return self.getTranslation(self.trackedTipNode)

  def getPlanningPoint(self, planningMarkupsNode, name='TARGET'):
    if planningMarkupsNode:
      idx = planningMarkupsNode.GetControlPointIndexByLabel(name)
      return list(planningMarkupsNode.GetNthControlPointPosition(idx))

  def getNumberOfPoints(self,planningMarkupsNode):
    if planningMarkupsNode is not None:
      return planningMarkupsNode.GetNumberOfDefinedControlPoints()
    else: 
      return 0
  
  def addPlanningPoint(self, planningMarkupsNode):
    if planningMarkupsNode is not None:
      if (planningMarkupsNode == self.robotPositionMarkupsNode) or (planningMarkupsNode == self.trackedTipMarkupsNode):
        print('Invalid point list')
        return
      N = planningMarkupsNode.GetNumberOfControlPoints()
      if N>=1:
        planningMarkupsNode.SetNthControlPointLabel(0, 'TARGET')
      if N>=2:
        planningMarkupsNode.SetNthControlPointLabel(1, 'ENTRY')

  def getUniqueNodeName(self, baseName, nodeClass="vtkMRMLLinearTransformNode"):
    if slicer.util.getFirstNodeByClassByName(nodeClass, baseName) is None:
      return baseName
    index = 1
    while True:
      candidateName = f"{baseName}_{index}"
      if slicer.util.getFirstNodeByClassByName(nodeClass, candidateName) is None:
        return candidateName
      index += 1

  # Place robot at the skin entry point
  def alignForInsertion(self, planningMarkupsNode):
    entry_scanner = [*self.getPlanningPoint(planningMarkupsNode, 'ENTRY'), 1.0]       # Get entry point in scanner coordinates
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner) # Convert to robot coordinates
    desired_position = [entry[0], 0.0, entry[2]]
    desired_position_scanner = self.mat_RobotToScanner.MultiplyPoint([desired_position[0], desired_position[1], desired_position[2], 1.0])
    print('Desired position = [%.4f, %.4f, %.4f] RAS' % (desired_position_scanner[0], desired_position_scanner[1], desired_position_scanner[2]))
    self.sendDesiredPosition(desired_position)
    return True
    
  # Place robot at the skin entry point
  def approachSkin(self, planningMarkupsNode):
    entry_scanner = [*self.getPlanningPoint(planningMarkupsNode, 'ENTRY'), 1.0]        # Get entry point in scanner coordinates
    print('Desired position = [%.4f, %.4f, %.4f] RAS' % (entry_scanner[0], entry_scanner[1], entry_scanner[2]))
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)  # Convert to robot coordinates
    self.sendDesiredPosition([entry[0], entry[1], entry[2]])
    return True

  def isRobotAligned(self, planningMarkupsNode):
    entry_scanner = [*self.getPlanningPoint(planningMarkupsNode, 'ENTRY'), 1.0] 
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)  # Convert to robot coordinates
    robot, _ = self.getRobotPosition()
    if (abs(robot[0] - entry[0]) <= self.eps) and (abs(robot[2] - entry[2]) <= self.eps):
      return True
    else:
      return False

  # Insert robot to target depth
  def toTarget(self, planningMarkupsNode):
    target_scanner = [*self.getPlanningPoint(planningMarkupsNode,'TARGET'), 1.0]
    print('Desired position = [%.4f, %.4f, %.4f] RAS' % (target_scanner[0], target_scanner[1], target_scanner[2]))
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    goal, _ = self.getRobotPosition()
    goal[1] = target[1]
    self.sendDesiredPosition(goal)
    return True

  # Insert robot by a step 
  def insertStep(self, stepSize):
    goal, _ = self.getRobotPosition()
    goal[1] += stepSize
    goal_scanner = self.mat_RobotToScanner.MultiplyPoint([goal[0], goal[1], goal[2], 1.0])
    print('Desired position = [%.4f, %.4f, %.4f] RAS' % (goal_scanner[0], goal_scanner[1], goal_scanner[2]))
    self.sendDesiredPosition(goal)
    return True
    
  def adjustEntry(self, planningMarkupsNode):
    # Get Points in scanner coordinates
    entry_scanner = [*self.getPlanningPoint(planningMarkupsNode,'ENTRY'), 1.0]
    target_scanner = [*self.getPlanningPoint(planningMarkupsNode,'TARGET'), 1.0]
    # Calculate Points in robot coordinates
    target = self.mat_ScannerToRobot.MultiplyPoint(target_scanner)
    entry = self.mat_ScannerToRobot.MultiplyPoint(entry_scanner)
    # Align entry with target (in robot coordinates)
    entry = [target[0], entry[1], target[2], 1.0]
    # Update entry scanner in markups list (to align with target)
    entry_scanner = self.mat_RobotToScanner.MultiplyPoint(entry)
    planningMarkupsNode.SetNthControlPointPosition(planningMarkupsNode.GetControlPointIndexByLabel('ENTRY'), entry_scanner[:3])
    planningMarkupsNode.SetLocked(True)
    return True

  def publishPlanning(self, planningMarkupsNode):
    # Get Points in scanner coordinates
    entry_scanner = [*self.getPlanningPoint(planningMarkupsNode,'ENTRY'), 1.0]
    target_scanner = [*self.getPlanningPoint(planningMarkupsNode,'TARGET'), 1.0]
    # Publish entry message
    entry_msg = self.pubEntry.GetBlankMessage()
    entry_msg.SetX(entry_scanner[0])
    entry_msg.SetY(entry_scanner[1])
    entry_msg.SetZ(entry_scanner[2])
    self.pubEntry.Publish(entry_msg)
    print('Published entry = [%.4f, %.4f, %.4f] RAS' % (entry_scanner[0], entry_scanner[1], entry_scanner[2]))
    # Publish target message
    target_msg = self.pubTarget.GetBlankMessage()
    target_msg.SetX(target_scanner[0])
    target_msg.SetY(target_scanner[1])
    target_msg.SetZ(target_scanner[2])
    self.pubTarget.Publish(target_msg)
    print('Published target = [%.4f, %.4f, %.4f] RAS' % (target_scanner[0], target_scanner[1], target_scanner[2]))

  def sendDesiredPosition(self, goal):
    # Publish desired_position message
    self.desired_position = goal
    goal_msg = self.pubGoal.GetBlankMessage()
    goal_msg.SetX(self.desired_position[0])
    goal_msg.SetY(self.desired_position[1])
    goal_msg.SetZ(self.desired_position[2])
    self.pubGoal.Publish(goal_msg)
    print('Desired position = [%.4f, %.4f, %.4f] XYZ' % (self.desired_position[0], self.desired_position[1], self.desired_position[2]))

  def sendDesiredCommand(self, command):
    # Publish desired_command message
    self.pubCommand.Publish(command)
  
  def updateTrackedTip(self):
    # Message incoming
    timestamp = time.time()
    dt = datetime.datetime.fromtimestamp(timestamp)
    logTimestamp = dt.strftime('%H:%M:%S.%f')[:-3]
    # Extract confidence 
    msg_text = self.needleConfidenceNode.GetText()
    parts = msg_text.split(';') # Split by ';'
    tipConfidence = parts[1].strip()
    confidenceValue = int(parts[2].strip())
    # Extract timestamp (from AITracking)
    timestamp = float(parts[0].strip())
    dt = datetime.datetime.fromtimestamp(timestamp)
    tipTimestamp = dt.strftime('%H:%M:%S.%f')[:-3]

    # Extract position
    tipPosition = self.getTipPosition()
    tipTracked = (confidenceValue >= 3)
    self.setTipMarkupColor(tipTracked)
    robotRAS, robotTimestamp = self.getRobotPosition('world')
    robotXYZ, _ = self.getRobotPosition()
    
    if tipTracked:
      print('Tracked')
      moduleParameterNode = self.getParameterNode()     # Update module parameter to help synch logic and gui
      moduleParameterNode.SetParameter('TipTracked', 'True')
      # Publish tracked_tip message
      tip_msg = self.pubTrackedTip.GetBlankMessage()
      tip_msg.SetX(tipPosition[0])
      tip_msg.SetY(tipPosition[1])
      tip_msg.SetZ(tipPosition[2])
      self.pubTrackedTip.Publish(tip_msg)

    
    # Update log
    if tipTracked and self.loggingActive:
      self.loggedRobotPositions.append(robotRAS)
      self.loggedRobotTimestamps.append(robotTimestamp)  
      self.loggedTipPositions.append(tipPosition)
      self.loggedTipTimestamps.append(tipTimestamp)    
      self.loggedLogTimestamps.append(logTimestamp)  
    print('Log Timestamp: %s' %(logTimestamp))
    if tipTracked:
      print('Tip = [%.4f, %.4f, %.4f] RAS, Confidence = %s, Tip Timestamp = %s' % (tipPosition[0], tipPosition[1], tipPosition[2], tipConfidence, tipTimestamp))
    else:
      print('Tip not tracked, Confidence = %s, Tip Timestamp = %s' % (tipConfidence, tipTimestamp))
    print('Robot = [%.4f, %.4f, %.4f] RAS, [%.4f, %.4f, %.4f] XYZ' % (robotRAS[0], robotRAS[1], robotRAS[2], robotXYZ[0], robotXYZ[1], robotXYZ[2]))
    print('____________________')

  def startLogging(self):
      self.loggingActive = True
      self.loggedRobotPositions = []    # Stored robot position (RAS) at tip receipt time
      self.loggedRobotTimestamps = []   # When robot position was stored
      self.loggedTipPositions = []      # Tip position
      self.loggedTipTimestamps = []     # From tip message header (other computer timestamp)
      self.loggedLogTimestamps = []     # When tip was received 

  def stopAndSaveLogging(self, planningNode):
      self.loggingActive = False
      # Extract insertion number from planning node name
      if planningNode is None:
        name = ''
      else:
        name = planningNode.GetName()
      insertion_number = ''.join(filter(str.isdigit, name))
      node_name = f'Insertion_{insertion_number}'

      # Save to CSV
      file_path = os.path.join(slicer.app.temporaryPath, f"{node_name}.csv")
      with open(file_path, 'w') as f:
          f.write("log_timestamp,tip_timestamp,tip_R,tip_A,tip_S,robot_timestamp_robot_R,robot_A,robot_S t\n")
          for ts_log, ts_tip, pos_tip, ts_robot, pos_robot  in zip(self.loggedLogTimestamps, self.loggedTipTimestamps, self.loggedTipPositions, self.loggedRobotTimestamps, self.loggedRobotPositions):
              f.write(f"{ts_log},{ts_tip},{pos_tip[0]:.3f},{pos_tip[1]:.3f},{pos_tip[2]:.3f},{ts_robot},{pos_robot[0]:.3f},{pos_robot[1]:.3f},{pos_robot[2]:.3f}\n")
      print(f"Saved insertion to: {file_path}")

      # Add to Markups Node
      node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", node_name)
      for pos in self.loggedTipPositions:
          node.AddControlPoint(vtk.vtkVector3d(*pos), "")
      node.CreateDefaultDisplayNodes()
