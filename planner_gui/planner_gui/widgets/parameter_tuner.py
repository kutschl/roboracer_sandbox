import sys, os
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QSplitter, QListWidget,
    QTreeView, QPushButton, QLineEdit, QLabel, QApplication,
    QHeaderView
)
from PyQt5.QtCore import Qt, QTimer, QThread
from PyQt5.QtGui import QStandardItemModel, QStandardItem
import asyncio
from functools import partial
from pathlib import Path

from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import yaml

# Import style definitions from your styles module.
from styles import *

class ParameterTunerWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.setWindowTitle('ROS 2 Parameter Tuner')
        self.setStyleSheet(f"background-color: rgb{str(container_bg_color)}; color: rgb{str(text_color)};")
        # ROS service node
        self._svc = node

        self.current_node = None
        # Local buffer for parameter changes.
        self._param_buffer = {}
        # Qt UI
        self._make_ui()
        self._refresh_timer = QTimer(self, interval=2000)
        self._refresh_timer.timeout.connect(self._refresh_nodes)
        self._refresh_timer.start()

    def _make_ui(self):
        main = QHBoxLayout(self)

        # ––––– Left: node list –––––
        self._nodes_list = QListWidget()
        self._nodes_list.setStyleSheet(listwidget_stylesheet)
        self._nodes_list.itemClicked.connect(self._on_node_selected)

        left = QVBoxLayout()
        left.addWidget(QLabel('Nodes:'))
        left.addWidget(self._nodes_list)
        left_widget = QWidget()
        left_widget.setLayout(left)
        left_widget.setStyleSheet(f"background-color: rgb{str(container_bg_color)};")

        # ––––– Right: parameter tree view –––––
        self._filter_edit = QLineEdit(placeholderText='Filter parameters')
        self._filter_edit.setStyleSheet(line_edit_stylesheet)
        self._filter_edit.textChanged.connect(self._reload_parameters)

        self._tree = QTreeView()
        # Append additional style to file_explorer_stylesheet to add separators and padding.
        self._tree.setStyleSheet(parameter_treeview_stylesheet)  # Remove parentheses from container_bg_color string if needed.
        self._model = QStandardItemModel()
        self._model.setHorizontalHeaderLabels(['Parameter', 'Value'])
        self._tree.setModel(self._model)
        self._tree.header().setStretchLastSection(True)
        # Give more width to the parameter column.
        self._tree.header().resizeSection(0, 300)

        self._btn_reload = QPushButton('Reload')
        self._btn_reload.setStyleSheet(button_stylesheet)
        self._btn_reload.clicked.connect(lambda *_: self._reload_parameters(False))

        self._btn_save = QPushButton('Save')
        self._btn_save.setStyleSheet(button_stylesheet)
        self._btn_save.clicked.connect(self._save_parameters)

        self._btn_deploy = QPushButton('Deploy')
        self._btn_deploy.setStyleSheet(button_stylesheet)
        self._btn_deploy.clicked.connect(self._deploy_parameters)

        right = QVBoxLayout()
        filt = QHBoxLayout()
        filt.addWidget(QLabel('Filter:'))
        filt.addWidget(self._filter_edit)
        right.addLayout(filt)
        right.addWidget(self._tree)
        # Add both Reload and Deploy buttons.
        btns = QHBoxLayout()
        btns.addWidget(self._btn_reload)
        btns.addWidget(self._btn_save)
        btns.addWidget(self._btn_deploy)
        right.addLayout(btns)
        right_widget = QWidget()
        right_widget.setLayout(right)
        right_widget.setStyleSheet(f"background-color: rgb{str(container_bg_color)};")

        splitter = QSplitter()
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(1, 1)

        main.addWidget(splitter)
        self.setLayout(main)

    def _refresh_nodes(self):
        self._nodes_list.clear()
        names = list(self._svc.query_nodes())
        names = sorted(names)
        for name in names:
            self._nodes_list.addItem(name)

    def _on_node_selected(self, item):
        self.current_node = item.text()
        self._svc.set_node_of_interest(item.text())
        self._reload_parameters()

    def _save_parameters(self):
        """
        Save the currently displayed parameters to a YAML configuration file.
        The file will be saved to a 'config' directory in the same folder as this file.
        """
        config = {}

        # Helper function to create cascading dictionaries from dot-separated keys.
        def set_nested_dict(d, keys, value):
            for key in keys[:-1]:
                d = d.setdefault(key, {})
            d[keys[-1]] = value

        # Traverse the model to extract parameters.
        def traverse(item):
            for row in range(item.rowCount()):
                key_item = item.child(row, 0)
                value_item = item.child(row, 1)
                # Use the full parameter name stored in custom role if available.
                param_name = value_item.data(Qt.UserRole + 1) or key_item.text()
                param_type = value_item.data(Qt.UserRole)
                if key_item.hasChildren():
                    # Recursively traverse child items.
                    traverse(key_item)
                else:
                    # Get the current value.
                    if param_type == 'bool':
                        value = (value_item.checkState() == Qt.Checked)
                    else:
                        value = self._format_param(value_item.text(), param_type)
                    # Split the parameter name by dots and create cascading dictionaries.
                    set_nested_dict(config, param_name.split('.'), value)

        root = self._model.invisibleRootItem()
        traverse(root)

        # Prepare the configuration file path.
        config_path = Path(os.environ["LARACE_ROOT"]) / "src" / "core" / "config" / os.environ["LARACE_CAR_VERSION"] / f"{self.current_node}.yaml"
        config = {self.current_node: {"ros__parameters": config}}
        try:
            with open(config_path, 'w') as f:
                yaml.dump(config, f)
            print(f"Parameters saved to {config_path}")
        except Exception as e:
            print(f"Error saving parameters: {e}")

    def _reload_parameters(self, collapse_all=True):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        print(f"Loading parameters for node: {self.current_node}")
        try:
            param_names = loop.run_until_complete(self._svc.list_parameters())
            if param_names is None:
                return
            # Sort alphabetically and apply filter:
            filt = self._filter_edit.text().lower()
            param_names = sorted([p for p in param_names if filt in p.lower()])

            # Create a new tree model.
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(['Parameter', 'Value'])

            # Helper function to recursively insert an item into the tree.
            def insert_parameter(root, parts, param_data):
                if not parts:
                    return
                key = parts[0]
                child = None
                for row in range(root.rowCount()):
                    item = root.child(row, 0)
                    if item.text() == key:
                        child = item
                        break
                if child is None:
                    child = QStandardItem(key)
                    # Create a placeholder for the value.
                    value_item = QStandardItem("")
                    root.appendRow([child, value_item])
                if len(parts) == 1:
                    # Leaf: set the value.
                    if param_data["type"] == "bool":
                        value_item = QStandardItem()
                        value_item.setCheckable(True)
                        value_item.setEditable(False)
                        value_item.setCheckState(Qt.Checked if param_data["display"] else Qt.Unchecked)
                        value_item.setData("bool", Qt.UserRole)
                    else:
                        value_item = QStandardItem(param_data["display"])
                        value_item.setData(param_data["type"], Qt.UserRole)
                    # Store the full parameter name in a custom role.
                    value_item.setData(param_data["full_name"], Qt.UserRole + 1)
                    # Replace the child's value column.
                    parent = child.parent() if child.parent() is not None else root
                    parent.setChild(child.row(), 1, value_item)
                else:
                    insert_parameter(child, parts[1:], param_data)

            # For each parameter, retrieve its value and insert it.
            for pname in param_names:
                values = loop.run_until_complete(self._svc.get_parameter_values([pname]))
                if not values:
                    display = '<no value>'
                    ptype = "other"
                else:
                    val_msg = values[0]
                    #print(val_msg)
                    if val_msg.type == ParameterType.PARAMETER_BOOL:
                        display = bool(val_msg.bool_value)
                        ptype = "bool"
                    elif val_msg.type == ParameterType.PARAMETER_INTEGER:
                        display = str(val_msg.integer_value)
                        ptype = "int"
                    elif val_msg.type == ParameterType.PARAMETER_DOUBLE:
                        display = str(val_msg.double_value)
                        ptype = "float"
                    elif val_msg.type == ParameterType.PARAMETER_STRING:
                        display = val_msg.string_value
                        ptype = "str"
                    else:
                        display = '<unhandled>'
                        ptype = "other"
                param_data = {"display": display, "type": ptype, "full_name": pname}
                #print(f"New parameter: {param_data}")

                parts = pname.split('.')
                insert_parameter(model.invisibleRootItem(), parts, param_data)

            self._model = model
            self._tree.setModel(self._model)
            self._tree.header().resizeSection(0, 300)
            # Collapse all branches by default.
            if collapse_all:
                self._tree.collapseAll()
            # Connect a signal to track changes in the value column.
            self._model.itemChanged.connect(self._on_item_changed)
        finally:
            loop.close()

    def _on_item_changed(self, item):
        # Only act on changes in the value column.
        if item.column() != 1:
            return
        # Retrieve the full parameter name from our custom role.
        full_name = item.data(Qt.UserRole + 1)
        if not full_name:
            return
        # Determine new value based on type.
        param_type = item.data(Qt.UserRole)
        if param_type == "bool":
            new_value = (item.checkState() == Qt.Checked)
        else:
            new_value = item.text()
            new_value = self._format_param(new_value, param_type)
        # Save change in local buffer.
        self._param_buffer[full_name] = new_value

    def _deploy_parameters(self):
        if not self._param_buffer:
            return
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        async def deploy_all():
            for pname, new_val in self._param_buffer.items():
                #print(f"Parameter change '{pname}' -> {new_val}")
                await self._svc.set_parameter(pname, new_val)
        loop.run_until_complete(deploy_all())
        loop.close()
        self._param_buffer.clear()
        print("Deployment complete.")

    def _format_param(self, param, type):
        if type == "bool":
            return bool(param)
        elif type == "int":
            return int(param)
        elif type == "float":
            return float(param)
        elif type == "str":
            return str(param)
        else:
            print(f"Unsupported parameter type: {type}")
            return param
        