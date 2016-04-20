public class GuiDialog {

    public GuiDialog(JFrame frame) {

        if(attributes.contains("Nothing")){}
            String[] temp = attributes.split("\\(");
            attributes = temp[1] + " " + temp[2];
            String[] optionsForTreasure= {"Take Picture", "Grab Treasure", "Continue"};
            final JOptionPane optionPane = new JOptionPane(
                                    attributes,
                                    JOptionPane.QUESTION_MESSAGE,
                                    JOptionPane.YES_NO_CANCEL_OPTION);

            optionPane.setOptions(optionsForTreasure);
            final JDialog dialog = new JDialog(frame,
                                         "Identified a shape",
                                         true);
            dialog.setContentPane(optionPane);
            dialog.setDefaultCloseOperation(
                JDialog.DO_NOTHING_ON_CLOSE);
            dialog.addWindowListener(new WindowAdapter() {
                public void windowClosing(WindowEvent we) {
                    System.out.println("Thwarted user attempt to close window.");
                }
            });
            optionPane.addPropertyChangeListener(
                new PropertyChangeListener() {
                    public void propertyChange(PropertyChangeEvent e) {
                        String prop = e.getPropertyName();

                        if (dialog.isVisible()
                         && (e.getSource() == optionPane)
                         && (JOptionPane.VALUE_PROPERTY.equals(prop))) {
                            dialog.setVisible(false);
                        }
                    }
                });
            dialog.pack();
            dialog.setLocationRelativeTo(frame);
            dialog.setVisible(true);

            String value = (String) optionPane.getValue();
            if (value.equals("Take Picture")) {
                takePicture(attributes);
            } else if (value.equals("Grab Treasure")) {
                System.out.println("Try using the window decorations "
                         + "to close the non-auto-closing dialog. "
                         + "You can't!");
            } else {
                
            }
    }
}