
DARK_MODE = True

# Darkmode setup
if DARK_MODE:
    text_color = (250,250,250)
    border_color = (20,20,20)
    container_bg_color = (30,30,30)
    tab_bg_color = border_color
    curr_lap_color = (255,0,255,255)
    last_lap_color = (200,0,200,200)
    best_lap_color = (0,200,0,200)
    ref_line_color = (0,200,200,200)
    alt_plot_color = (150,150,150,255)
    change_plot_color = (255,35,45,255)
    btn_bg_color = (50,50,50)
else:
    # TODO: white mode coloring
    text_color = 'black'
    border_color = 'rgba(200, 200, 200, 100)'
    container_bg_color = 'white'
    tab_bg_color = 'rgba(240, 240, 240, 100)'

plot_stylesheet = f"""
        border: none;
"""

title_stylesheet = f"""
    color: rgb{str(text_color)}; 
    font-size: 12pt;
    font-weight: bold;
    background-color: rgb{str(container_bg_color)};
    border: none;
"""

button_stylesheet = f"""
            QPushButton {{
                background-color: rgb{btn_bg_color}; 
                font-size: 12px;
                color: rgb{text_color};
                border: 1px solid rgb{text_color};
                border-radius: 2px;
                padding: 6px 12px;
            }}
            QPushButton:hover {{
                background-color: rgb{tuple(min(int(c * 1.25), 255) for c in btn_bg_color)};
                color: white;
            }}
            QPushButton:pressed {{
                background-color: rgb{tuple(min(int(c * 1.75), 255) for c in btn_bg_color)};
                color: white;
            }}
        """

button_thin_stylesheet = f"""
            QPushButton {{
                background-color: rgb{btn_bg_color}; 
                font-size: 12px;
                color: rgb{text_color};
                border: 1px solid rgb{text_color};
                border-radius: 2px;
                padding: 0px 0px;
            }}
            QPushButton:hover {{
                background-color: rgb{tuple(min(int(c * 1.25), 255) for c in btn_bg_color)};
                color: white;
            }}
            QPushButton:pressed {{
                background-color: rgb{tuple(min(int(c * 1.75), 255) for c in btn_bg_color)};
                color: white;
            }}
        """

plain_text_edit_stylesheet = f"""
    QPlainTextEdit {{
         background-color: rgb(50,50,50);
         color: rgb(250,250,250);
         font-size: 12px;
         border: 1px solid rgb(50,50,50);
         padding: 4px;
    }}
    QPlainTextEdit:focus {{
         background-color: rgb(60,60,60);
         border: 1px solid rgb(250,250,250);
    }}
    /* Vertical scrollbar styling */
    QPlainTextEdit QScrollBar:vertical {{
         background: rgb(30,30,30);
         width: 16px;
         margin: 0px;
    }}
    QPlainTextEdit QScrollBar::handle:vertical {{
         background: rgb(50,50,50);
         border: 1px solid rgb(50,50,50);
         border-radius: 4px;
         min-height: 10px;
    }}
    QPlainTextEdit QScrollBar::add-line:vertical, QPlainTextEdit QScrollBar::sub-line:vertical {{
         height: 0;
         subcontrol-origin: margin;
    }}
    QPlainTextEdit QScrollBar::add-page:vertical, QPlainTextEdit QScrollBar::sub-page:vertical {{
         background: none;
    }}
    /* Horizontal scrollbar styling */
    QPlainTextEdit QScrollBar:horizontal {{
         background: rgb(30,30,30);
         height: 16px;
         margin: 0px;
    }}
    QPlainTextEdit QScrollBar::handle:horizontal {{
         background: rgb(50,50,50);
         border: 1px solid rgb(50,50,50);
         border-radius: 4px;
         min-width: 10px;
    }}
    QPlainTextEdit QScrollBar::add-line:horizontal, QPlainTextEdit QScrollBar::sub-line:horizontal {{
         width: 0;
         subcontrol-origin: margin;
    }}
    QPlainTextEdit QScrollBar::add-page:horizontal, QPlainTextEdit QScrollBar::sub-page:horizontal {{
         background: none;
    }}
"""

checkbox_stylesheet = f"""
    QCheckBox {{
        background-color: rgb{btn_bg_color};
        font-size: 12px;
        color: rgb{text_color};
    }}
    
    QCheckBox::indicator {{
        width: 14px;
        height: 14px;
        border-radius: 2px;
        border: 1px solid rgb{text_color};
        background-color: rgb{btn_bg_color};
    }}

    QCheckBox::indicator:hover {{
        background-color: rgb{tuple(min(int(c * 1.5), 255) for c in btn_bg_color)};
        border: 1px solid white;
    }}

    QCheckBox::indicator:checked {{
        background-color: rgb{tuple(min(int(c * 3.0), 255) for c in btn_bg_color)};
        border: 1px solid white;
    }}
"""
tab_stylesheet = f"""
    QTabWidget::pane {{
        border: 1px solid rgb(50,50,50);
        background-color: rgb{str(container_bg_color)};
    }}
    QTabBar::tab {{
        background-color: rgb(50,50,50);
        color: rgb{str(text_color)};
        padding: 6px 12px;
        border: 1px solid rgb(50,50,50);
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        margin-right: 2px;
    }}
    QTabBar::tab:selected {{
        background-color: rgb(60,60,60);
        border-bottom: 1px solid rgb(60,60,60);
    }}
    QTabBar::tab:hover {{
        background-color: rgb(62,62,62);
    }}
    QTabBar::tab:!selected {{
        margin-top: 2px;
    }}
"""

listwidget_stylesheet = f"""
    QListWidget {{
        background-color: rgb{str(btn_bg_color)};
        color: rgb{str(text_color)};
        border: 1px solid rgb(50,50,50);
        padding: 4px;
        font-size: 12px;
    }}
    QListWidget::item {{
        padding: 4px 2px;
        border-bottom: 1px solid rgb(70,70,70);
    }}
    QListWidget::item:selected {{
        background-color: rgb(60,60,60);
        color: rgb(250,250,250);
    }}
    QListWidget::item:hover {{
        background-color: rgb(62,62,62);
    }}
"""

file_explorer_stylesheet = f"""
    QTreeView {{
        background-color: rgb(30,30,30);
        color: rgb(250,250,250);
        border: 1px solid rgb(50,50,50);
        gridline-color: rgb(50,50,50);
        font-size: 12px;
    }}
    QTreeView::item {{
        padding: 1px;
    }}
    QTreeView::item:selected {{
        background-color: rgb(50,50,50);
    }}

    /* Style the header */
    QHeaderView::section {{
        background-color: rgb(50,50,50);
        color: rgb(250,250,250);
        padding: 4px;
        border: 1px solid rgb(50,50,50);
    }}

    /* Vertical scrollbar styling */
    QScrollBar:vertical {{
        background: rgb(30,30,30);
        width: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:vertical {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-height: 10px;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
        background: none;
    }}

    /* Horizontal scrollbar styling */
    QScrollBar:horizontal {{
        background: rgb(30,30,30);
        height: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:horizontal {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-width: 10px;
    }}
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
        background: none;
    }}
"""

scrollarea_stylesheet = f"""
    QScrollArea {{
         background-color: rgb{str(container_bg_color)};
         border: 1px solid rgb(50,50,50);
         padding: 0px;
    }}
    QScrollArea > QWidget {{
         background-color: rgb{str(container_bg_color)};
         color: rgb{str(text_color)};
    }}
    QScrollArea QFrame {{
         background-color: rgb{str(container_bg_color)};
         border: none;
    }}
"""

container_label_stylesheet = f"""
    QLabel {{
        background-color: rgb{str(tab_bg_color)};
        color: rgb{str(text_color)};
        font-size: 12px;
        padding: 4px;
    }}
"""

heading_label_stylesheet = f"""
    QLabel {{
        background-color: rgb{str(tab_bg_color)};
        color: rgb{str(text_color)};
        font-size: 24px;
        font-weight: bold;
        padding: 0px 0px;
    }}
"""

menu_bar_stylesheet = f"""
    QMenuBar {{
         background-color: rgb{str(container_bg_color)};
         color: rgb{str(text_color)};
         border-bottom: 1px solid rgb(50,50,50);
         padding: 4px;
    }}
    QMenuBar::item {{
         background: transparent;
         padding: 4px 10px;
         margin: 0px;
    }}
    QMenuBar::item:selected {{
         background-color: rgb(50,50,50);
    }}
    QMenuBar::item:pressed {{
         background-color: rgb(60,60,60);
    }}
    QMenu {{
         background-color: rgb{str(container_bg_color)};
         border: 1px solid rgb(50,50,50);
         color: rgb{str(text_color)};
    }}
    QMenu::item {{
         padding: 4px 20px;
    }}
    QMenu::item:selected {{
         background-color: rgb(50,50,50);
    }}
"""

scrollbar_stylesheet = f"""
    /* Vertical scrollbar styling */
    QScrollBar:vertical {{
        background: rgb(30,30,30);
        width: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:vertical {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-height: 20px;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
        background: none;
    }}

    /* Horizontal scrollbar styling */
    QScrollBar:horizontal {{
        background: rgb(30,30,30);
        height: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:horizontal {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-width: 20px;
    }}
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
        background: none;
    }}
"""

combobox_stylesheet = f"""
    QComboBox {{
        background-color: rgb(50,50,50);
        color: rgb(250,250,250);
        font-size: 12px;
        border: 1px solid rgb(250,250,250);
        border-radius: 2px;
        padding: 6px 12px;
    }}
    QComboBox:hover {{
        background-color: rgb(62,62,62);
    }}
    QComboBox::drop-down {{
        subcontrol-origin: padding;
        subcontrol-position: top right;
        width: 20px;
        border-left: 1px solid rgb(250,250,250);
    }}
    QComboBox::down-arrow {{
        /* Replace this URL with an appropriate down-arrow icon path if needed */
        image: url(:/icons/down-arrow.png);
        width: 10px;
        height: 10px;
    }}
    /* Style the popup list */
    QComboBox QAbstractItemView {{
        background-color: rgb(30,30,30);
        color: rgb(250,250,250);
        border: 1px solid rgb(50,50,50);
        selection-background-color: rgb(62,62,62);
        outline: none;
        margin: 0;      /* Remove margin to eliminate extra white space */
        padding: 0;     /* Remove padding as well */
    }}
    QComboBox QAbstractItemView::item {{
        padding: 4px 8px;
    }}
"""

line_edit_stylesheet = f"""
    QLineEdit {{
        background-color: rgb(50, 50, 50);
        color: rgb(250, 250, 250);
        font-size: 12px;
        border: 1px solid rgb(250, 250, 250);
        border-radius: 2px;
        padding: 6px 12px;
    }}
    QLineEdit:focus {{
        background-color: rgb(60, 60, 60);
        border: 1px solid rgb{tuple(min(int(c * 1.25), 255) for c in (50, 50, 50))};
    }}
"""

table_stylesheet = """
    QTableWidget {
        background-color: transparent;
        color: white;
        font-size: 10pt;
    }
    QTableWidget::item {
        border: none;
        padding: 4px;
    }
"""

data_stylesheet = f"""
    color: rgb{str(text_color)};
    font-size: 10pt;
    background-color: rgb{str(container_bg_color)};
    font-family: NotoMono;
    border: none;
"""
tab_stylesheet = f"""
    background-color: rgb{tab_bg_color};
"""

tab_widget_stylesheet = f"""
    QTabWidget::pane {{
        border: 1px solid rgb(50,50,50);
        background-color: rgb{str(container_bg_color)};
    }}
    QTabBar::tab {{
        background-color: rgb(50,50,50);
        color: rgb{str(text_color)};
        padding: 6px 12px;
        border: 1px solid rgb(50,50,50);
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        margin-right: 2px;
    }}
    QTabBar::tab:selected {{
        background-color: rgb(60,60,60);
        border-bottom: 1px solid rgb(60,60,60);
    }}
    QTabBar::tab:hover {{
        background-color: rgb(62,62,62);
    }}
    QTabBar::tab:!selected {{
        margin-top: 2px;
    }}
"""

slider_stylesheet = f"""
QSlider::groove:horizontal {{
    border: 1px solid rgb{str(text_color)};
    height: 8px;
    background: rgb{str(container_bg_color)};
    margin: 0px;
    border-radius: 4px;
}}

QSlider::handle:horizontal {{
    background: rgb{str(btn_bg_color)};
    border: 1px solid rgb{str(text_color)};
    width: 18px;
    margin: -5px 0; /* Overlap handle on groove */
    border-radius: 9px;
}}

QSlider::groove:vertical {{
    border: 1px solid rgb{str(text_color)};
    width: 8px;
    background: rgb{str(container_bg_color)};
    margin: 0px;
    border-radius: 4px;
}}

QSlider::handle:vertical {{
    background: rgb{str(btn_bg_color)};
    border: 1px solid rgb{str(text_color)};
    height: 18px;
    margin: 0 -5px; /* Overlap handle on groove */
    border-radius: 9px;
}}

QSlider::handle:hover {{
    background: rgb{tuple(min(int(c * 1.25), 255) for c in btn_bg_color)};
    border: 1px solid white;
}}
"""

bar_plot_stylesheet = f"""
    background-color: rgb{str(container_bg_color)};
    color: rgb{str(text_color)};
    border: none;
    font-family: NotoMono;
    font-size: 10pt;
"""

parameter_treeview_stylesheet = f"""
    QTreeView {{
        background-color: rgb{str(btn_bg_color)};
        alternate-background-color: rgb(100, 100, 100);
        color: rgb(250, 250, 250);
        border: 1px solid rgb(50, 50, 50);
        gridline-color: rgb(50, 50, 50);
        font-size: 12px;
    }}
    QTreeView::item {{
        padding: 3px 1px;
    }}
    QTreeView::item:selected {{
        background-color: rgb{str(btn_bg_color)};
    }}

    /* Style the header */
    QHeaderView::section {{
        background-color: rgb{str(btn_bg_color)};
        color: rgb(250, 250, 250);
        padding: 4px;
        border: 1px solid rgb(50, 50, 50);
        border-bottom: 1px solid rgb(80, 80, 80);
    }}

    /* Vertical scrollbar styling */
    QScrollBar:vertical {{
        background: rgb(30,30,30);
        width: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:vertical {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-height: 10px;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
        background: none;
    }}

    /* Horizontal scrollbar styling */
    QScrollBar:horizontal {{
        background: rgb(30,30,30);
        height: 16px;
        margin: 0px;
    }}
    QScrollBar::handle:horizontal {{
        background: rgb(50,50,50);
        border: 1px solid rgb(50,50,50);
        border-radius: 4px;
        min-width: 10px;
    }}
    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0;
        subcontrol-origin: margin;
    }}
    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
        background: none;
    }}
"""