import java.awt.Component;

import javax.swing.AbstractCellEditor;
import javax.swing.JComponent;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.JTextArea;
import javax.swing.JPanel;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableColumn;
import javax.swing.JLabel;
import javax.swing.JProgressBar;
import java.awt.*;
import javax.swing.BorderFactory;


class RobotRenderer implements TableCellRenderer
{
    private JPanel panel;
    private JPanel subPanel;
    private JLabel robotName;
    private JLabel robotAt;
    private JLabel energy;
    private JProgressBar energyLevel;

    public RobotRenderer()
    {
        panel = new JPanel(new BorderLayout());
        robotName = new JLabel();
        robotAt = new JLabel();
        energy = new JLabel();
        energyLevel = new JProgressBar(0,100);
        subPanel = new JPanel(new BorderLayout());
    }

    public Component getTableCellRendererComponent(
        JTable table, Object value, boolean isSelected,
        boolean hasFocus, final int row, final int column)
    {
        panel.removeAll();
        panel.setLayout(new BorderLayout());
        if (isSelected)
            panel.setBackground( table.getSelectionBackground() );
        else
            panel.setBackground( table.getBackground() );

        if (value == null
        ||  value.toString().length() == 0)
            return panel;
        Robot temp = (Robot) value;

        String robotNM = temp.name;
        String robotLoc = temp.location;
        int enrg = temp.energyLeft;

        robotName.setText( robotNM );
        robotAt.setText( "Location: " + robotLoc );
        energyLevel.setValue(enrg);
        energyLevel.setStringPainted(true);
        energy.setText("Energy Left: ");

        panel.add(robotName, BorderLayout.NORTH);
        panel.add(robotAt, BorderLayout.CENTER);
        subPanel.add(energy, BorderLayout.NORTH);
        subPanel.add(energyLevel, BorderLayout.SOUTH);
        panel.add(subPanel, BorderLayout.SOUTH);
        panel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        return panel;
    }
}