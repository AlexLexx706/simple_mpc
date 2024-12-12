import logging
import sys
from PyQt5 import QtWidgets
from mpc_view import view


logging.getLogger(__name__)


def main():
    """Display of MPC View
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(levelname)s:%(funcName)s: %(message)s'
    )
    logging.getLogger('PyQt5').setLevel(logging.WARNING)

    app = QtWidgets.QApplication(sys.argv)
    widget = view.View()
    widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
