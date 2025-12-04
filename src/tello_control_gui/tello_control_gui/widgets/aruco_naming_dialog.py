"""ArUco marker naming dialog for managing marker names."""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


class ArucoNamingDialog(QDialog):
    """Dialog untuk menamai ArUco marker IDs.
    
    Dialog ini memungkinkan pengguna untuk memberikan nama custom
    pada setiap ID marker ArUco yang terdeteksi. Nama ini akan
    ditampilkan di video overlay dan pada PDF export.
    """
    
    names_updated = pyqtSignal(dict)  # Emits {marker_id: name} dict
    
    def __init__(self, marker_names: dict = None, parent=None):
        """Initialize the ArUco naming dialog.
        
        Args:
            marker_names: Existing marker name mapping {id: name}
            parent: Parent widget
        """
        super().__init__(parent)
        
        self.marker_names = marker_names.copy() if marker_names else {}
        
        self.setWindowTitle("ArUco Marker Names")
        self.setMinimumSize(400, 400)
        self.setStyleSheet("""
            QDialog {
                background-color: #1a1a2e;
                color: white;
            }
            QLabel {
                color: white;
            }
            QLineEdit {
                background-color: #2a2a3e;
                border: 1px solid #444;
                border-radius: 4px;
                padding: 6px;
                color: white;
            }
            QLineEdit:focus {
                border: 1px solid #00BCD4;
            }
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
            QTableWidget {
                background-color: #2a2a3e;
                border: 1px solid #444;
                border-radius: 4px;
                gridline-color: #444;
                color: white;
            }
            QTableWidget::item {
                padding: 5px;
            }
            QTableWidget::item:selected {
                background-color: #00BCD4;
            }
            QHeaderView::section {
                background-color: #333;
                color: white;
                padding: 6px;
                border: none;
                border-right: 1px solid #444;
                font-weight: bold;
            }
            QGroupBox {
                font-weight: bold;
                color: #00BCD4;
                border: 1px solid #333;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        
        self.setup_ui()
        self.load_existing_names()
    
    def setup_ui(self):
        """Setup the dialog UI."""
        layout = QVBoxLayout()
        layout.setSpacing(12)

        # Add new marker section
        add_group = QGroupBox("Add New Marker")
        add_layout = QHBoxLayout()
        
        self.id_input = QLineEdit()
        self.id_input.setPlaceholderText("Marker ID (number)")
        self.id_input.setFixedWidth(100)
        
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Marker name (e.g., 'Takeoff Point')")
        
        self.add_btn = QPushButton("Add")
        self.add_btn.setFixedWidth(80)
        self.add_btn.clicked.connect(self.add_marker)
        
        add_layout.addWidget(QLabel("ID:"))
        add_layout.addWidget(self.id_input)
        add_layout.addWidget(QLabel("Name:"))
        add_layout.addWidget(self.name_input)
        add_layout.addWidget(self.add_btn)
        
        add_group.setLayout(add_layout)
        layout.addWidget(add_group)
        
        # Marker table
        table_group = QGroupBox("Saved Markers")
        table_layout = QVBoxLayout()
        
        self.marker_table = QTableWidget()
        self.marker_table.setColumnCount(3)
        self.marker_table.setHorizontalHeaderLabels(["ID", "Name", "Actions"])
        self.marker_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.marker_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.marker_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Fixed)
        self.marker_table.setColumnWidth(0, 60)
        self.marker_table.setColumnWidth(2, 80)
        self.marker_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.marker_table.verticalHeader().setVisible(False)
        
        table_layout.addWidget(self.marker_table)
        table_group.setLayout(table_layout)
        layout.addWidget(table_group, stretch=1)
        
        # Button row
        btn_layout = QHBoxLayout()
        
        self.clear_btn = QPushButton("Clear All")
        self.clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
            }
            QPushButton:hover {
                background-color: #D32F2F;
            }
        """)
        self.clear_btn.clicked.connect(self.clear_all)
        
        self.save_btn = QPushButton("Save & Close")
        self.save_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
        """)
        self.save_btn.clicked.connect(self.save_and_close)
        
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #666;
            }
            QPushButton:hover {
                background-color: #888;
            }
        """)
        self.cancel_btn.clicked.connect(self.reject)
        
        btn_layout.addWidget(self.clear_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addWidget(self.save_btn)
        
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)
    
    def load_existing_names(self):
        """Load existing marker names into the table."""
        self.marker_table.setRowCount(0)
        
        for marker_id, name in sorted(self.marker_names.items(), key=lambda x: x[0]):
            self._add_table_row(marker_id, name)
    
    def _add_table_row(self, marker_id: int, name: str):
        """Add a row to the marker table."""
        row = self.marker_table.rowCount()
        self.marker_table.insertRow(row)
        
        # ID column
        id_item = QTableWidgetItem(str(marker_id))
        id_item.setTextAlignment(Qt.AlignCenter)
        id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
        self.marker_table.setItem(row, 0, id_item)
        
        # Name column (editable)
        name_item = QTableWidgetItem(name)
        self.marker_table.setItem(row, 1, name_item)
        
        # Delete button
        delete_btn = QPushButton("🗑️")
        delete_btn.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #F44336;
                font-size: 14px;
                border: none;
            }
            QPushButton:hover {
                color: #FF6B6B;
            }
        """)
        delete_btn.clicked.connect(lambda _, r=row, mid=marker_id: self.delete_marker(mid))
        self.marker_table.setCellWidget(row, 2, delete_btn)
    
    def add_marker(self):
        """Add a new marker from input fields."""
        try:
            marker_id = int(self.id_input.text().strip())
            name = self.name_input.text().strip()
            
            if not name:
                QMessageBox.warning(self, "Error", "Please enter a name for the marker.")
                return
            
            if marker_id < 0:
                QMessageBox.warning(self, "Error", "Marker ID must be a positive number.")
                return
            
            # Check if already exists
            if marker_id in self.marker_names:
                reply = QMessageBox.question(
                    self, "Confirm Overwrite",
                    f"Marker ID {marker_id} already has a name.\nOverwrite with '{name}'?",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply != QMessageBox.Yes:
                    return
                # Find and update existing row
                for row in range(self.marker_table.rowCount()):
                    if int(self.marker_table.item(row, 0).text()) == marker_id:
                        self.marker_table.item(row, 1).setText(name)
                        break
            else:
                self._add_table_row(marker_id, name)
            
            self.marker_names[marker_id] = name
            
            # Clear inputs
            self.id_input.clear()
            self.name_input.clear()
            self.id_input.setFocus()
            
        except ValueError:
            QMessageBox.warning(self, "Error", "Please enter a valid marker ID (number).")
    
    def delete_marker(self, marker_id: int):
        """Delete a marker from the list."""
        if marker_id in self.marker_names:
            del self.marker_names[marker_id]
        
        # Remove from table
        for row in range(self.marker_table.rowCount()):
            if int(self.marker_table.item(row, 0).text()) == marker_id:
                self.marker_table.removeRow(row)
                break
    
    def clear_all(self):
        """Clear all marker names."""
        reply = QMessageBox.question(
            self, "Confirm Clear",
            "Are you sure you want to clear all marker names?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.marker_names.clear()
            self.marker_table.setRowCount(0)
    
    def save_and_close(self):
        """Save marker names and close dialog."""
        # Update names from table (in case user edited them directly)
        for row in range(self.marker_table.rowCount()):
            marker_id = int(self.marker_table.item(row, 0).text())
            name = self.marker_table.item(row, 1).text().strip()
            if name:
                self.marker_names[marker_id] = name
        
        self.names_updated.emit(self.marker_names)
        self.accept()
    
    def get_marker_names(self) -> dict:
        """Get the current marker names dictionary."""
        return self.marker_names.copy()
