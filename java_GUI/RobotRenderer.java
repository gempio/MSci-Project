import java.awt.Component;

import javax.swing.AbstractCellEditor;
import javax.swing.JComponent;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.JTextArea;
import javax.swing.JPanel;
import javax.swing.table.TableCellEditor;
import javax.swing.table.TableColumn;

class RobotRenderer extends AbstractCellEditor implements TableCellEditor {

  JPanel component = new JPanel();

  public Component getTableCellEditorComponent(JTable list, Object value, boolean isSelected,
      int rowIndex, int vColIndex) {

  	Robot tempRobot = (Robot) value;
  	System.out.println(tempRobot.name);
  	System.out.println(tempRobot.energyLeft);
  	JTextArea name = new JTextArea(tempRobot.name);
  	JTextArea energy = new JTextArea("" + tempRobot.energyLeft);
  	component.add(name);
  	component.add(energy);

    return component;
  }

  public Object getCellEditorValue() {
    return component.toString();
  }
}
