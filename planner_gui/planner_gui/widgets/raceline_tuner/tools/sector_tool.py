from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QHeaderView, QPushButton
)
from PyQt5.QtCore import Qt
from styles import table_stylesheet, button_stylesheet

class SectorTableWidget(QWidget):
    def __init__(self, parent_tuner):
        super().__init__()
        self.parent_tuner = parent_tuner
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        # table: change column count from 10 to 11 and add new header "force_fallback"
        self.table = QTableWidget(0, 11)
        self.table.setHorizontalHeaderLabels([
            "id","start","end","v_max","fallback","overtaking",
            "l1_scale","l1_shift","l1_min","l1_max", "force_fallback"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setStyleSheet(table_stylesheet)
        layout.addWidget(self.table)

        # accept button (implementation TBD)
        self.accept_btn = QPushButton("Accept")
        self.accept_btn.setStyleSheet(button_stylesheet)
        self.accept_btn.clicked.connect(self.accept)
        layout.addWidget(self.accept_btn)

    def clear_sectors(self):
        """Empty out the table and reset headers."""
        self.table.clearContents()
        self.table.setRowCount(0)
        # update headers to include "force_fallback"
        self.table.setHorizontalHeaderLabels([
            "id","start","end","v_max","fallback","overtaking",
            "l1_scale","l1_shift","l1_min","l1_max", "force_fallback"
        ])

    def update_table(self, sectors):
        """
        Repopulate the table from self.parent_tuner.sectors.
        Supports either tuple-based sectors [(id,start,end), …]
        or dict-based sectors [{'sector':…, 'start':…, …}, …].
        """
        self.clear_sectors()
        #print(sectors)
        #import ipdb; ipdb.set_trace()
        for i, sec in enumerate(sectors):
            row = self.table.rowCount()
            self.table.insertRow(row)

            # unpack either dict or tuple
            if isinstance(sec, dict):
                sid        = sec.get('sector', sec.get('id', i+1))
                start_idx  = sec.get('start', '')
                end_idx    = sec.get('end', '')
                vmax       = sec.get('v_max', '')
                fallback   = sec.get('fallback', False)
                overtaking = sec.get('overtaking', False)
                l1_scale   = sec.get('l1_scale', '')
                l1_shift   = sec.get('l1_shift', '')
                l1_min     = sec.get('l1_min', '')
                l1_max     = sec.get('l1_max', '')
                force_fb   = sec.get('force_fallback', False)
            else:
                # assume (id, start, end)
                sid, start_idx, end_idx = sec[:3]
                vmax = ''
                fallback = False
                overtaking = False
                l1_scale = l1_shift = l1_min = l1_max = ''
                force_fb = False

            values = [
                sid, start_idx, end_idx,
                vmax, fallback, overtaking,
                l1_scale, l1_shift, l1_min, l1_max, force_fb
            ]

            for col, val in enumerate(values):
                # columns 4, 5, and now 10 (fallback, overtaking, force_fallback) as checkboxes
                if col in (4, 5, 10):
                    item = QTableWidgetItem()
                    item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
                    item.setCheckState(Qt.Checked if bool(val) else Qt.Unchecked)
                else:
                    item = QTableWidgetItem(str(val))
                    flags = Qt.ItemIsSelectable | Qt.ItemIsEnabled
                    # make everything but the Sector‐ID column editable
                    if col != 0:
                        flags |= Qt.ItemIsEditable
                    item.setFlags(flags)

                self.table.setItem(row, col, item)

    def table_to_dict(self):
        """
        Convert the table data to a list of dictionaries.
        Each dictionary represents a sector with its properties.
        """
        sectors = []
        for row in range(self.table.rowCount()):
            sector = {}
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                header = self.table.horizontalHeaderItem(col).text()
                if item is not None:
                    if col in (4, 5, 10):
                        # convert check state to boolean
                        sector[header] = item.checkState() == Qt.Checked
                    else:
                        sector[header] = float(item.text())
            sectors.append(sector)
        return sectors

    def accept(self):
        sectors = self.table_to_dict()
        self.parent_tuner.raceline.set_sectors_from_dict(sectors)
        pass