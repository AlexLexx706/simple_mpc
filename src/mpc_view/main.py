import logging
import sys
from PyQt5 import QtWidgets
from mpc_view import main_widget


logging.getLogger(__name__)


def main():
    """Main function
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(levelname)s:%(funcName)s: %(message)s'
    )
    logging.getLogger('PyQt5').setLevel(logging.WARNING)

    app = QtWidgets.QApplication(sys.argv)
    view = main_widget.MainWidget()
    view.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
