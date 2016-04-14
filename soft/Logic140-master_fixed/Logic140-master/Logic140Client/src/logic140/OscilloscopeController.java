/*
 * Copyright (c) 2015, Andrey Petushkov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package logic140;

import javafx.beans.binding.Bindings;
import javafx.beans.binding.NumberBinding;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.DoublePropertyBase;
import javafx.beans.value.ObservableValue;
import javafx.scene.Cursor;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.TableColumn;
import javafx.scene.effect.Bloom;
import javafx.scene.effect.Glow;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Line;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Shape;
import javafx.scene.shape.StrokeLineCap;
import javafx.scene.transform.Transform;
import javafx.stage.Stage;
import java.util.Arrays;

/**
 *
 * @author Andrey Petushkov
 */
public class OscilloscopeController implements MainController.IController {
    private final int waveHandleWidth = 10;
    private final int waveHandleHeight = 10;

    private class ZeroLevelProperty extends DoublePropertyBase {

        @Override
        public Object getBean() {
            return this;
        }

        @Override
        public String getName() {
            return "zeroLevelProperty";
        }

        @Override
        protected void invalidated() {
            mController.redrawWaves();
        }
    };
    
    private final MainController mController;
    private Data dataRef;
    private Data.DataIterator dataIterator;
    private Canvas wave;
    private final ZeroLevelProperty ch1Zero = new ZeroLevelProperty();
    private final ZeroLevelProperty ch2Zero = new ZeroLevelProperty();
    private final Shape handleCh1 = new Rectangle(waveHandleWidth, waveHandleHeight);
    private final Shape handleCh2 = new Rectangle(waveHandleWidth, waveHandleHeight);
    private final Line axisXCh1 = new Line();
    private final Line axisXCh2 = new Line();
    private Object draggingHandle;
    
    public OscilloscopeController(MainController mController) {
        this.mController = mController;
        dataRef = Main.oData;
        dataIterator = dataRef.new DataIterator();
    }
    
    void startImpl() {
        mController.goButton2.setText("Running, click to S_top");
        mController.loadButton2.setDisable(true);
        mController.saveButton2.setDisable(false);
        mController.saveButton.setDisable(true);
        mController.loadButton.setDisable(true);
        mController.goButton.setDisable(true);
        mController.capturedInfoLabel2.setText("last capture at "+mController.freqChoice2.getSelectionModel().getSelectedItem());

        Main.setVoltage(DDS140.Voltage.values()[mController.ch1VoltChoiceBox.getSelectionModel().getSelectedIndex()], 
                DDS140.Voltage.values()[mController.ch2VoltChoiceBox.getSelectionModel().getSelectedIndex()]);
        Main.startAcquisition(DDS140.Frequency.values()[mController.freqChoice2.getSelectionModel().getSelectedIndex()], false, false);

        mController.updateZoomFactorTimeMode();
    }
    
    @Override
    public void setStoppedState() {
        mController.goButton2.setSelected(false);
        mController.goButton2.setText("_Go");
        mController.goButton.setDisable(false);
        mController.loadButton.setDisable(false);
        mController.capturedInfoLabel2.setText(mController.capturedInfoLabel2.getText()+
                String.format(" (%d%% in)", (int)dataRef.percentCaptured));
        mController.loadButton2.setDisable(false);
    }

    @Override
    public void initControls() {
        DDS140.Voltage[] voltages = DDS140.Voltage.values();
        for (DDS140.Voltage v: voltages) {
            mController.ch1VoltChoiceBox.getItems().add(v.getName());
            mController.ch2VoltChoiceBox.getItems().add(v.getName());
        }
        mController.ch1VoltChoiceBox.getSelectionModel().select(voltages.length-1);
        mController.ch2VoltChoiceBox.getSelectionModel().select(voltages.length-1);
        mController.log2TimeColumn.setCellValueFactory((TableColumn.CellDataFeatures<Event, Number> param) -> param.getValue().timeProperty);
        mController.log2EventColumn.setCellValueFactory((TableColumn.CellDataFeatures<Event, String> param) -> param.getValue().eventProperty);
        mController.logTable2.setItems(Main.oEvents);
        
        String[] acdc = {"AC", "DC"};
        mController.ch1AcDcChoiceBox.getItems().addAll(acdc);
        mController.ch2AcDcChoiceBox.getItems().addAll(acdc);
        
        mController.goButton2.disableProperty().bind(Bindings.not(Bindings.and(mController.goButton2Enable, 
                Bindings.or(mController.ch1EnableToggleButton.selectedProperty(), mController.ch2EnableToggleButton.selectedProperty()))));
        mController.goButton2Enable.set(true);
    }
    
    @Override
    public void postInitControls(Stage stage) {
        wave = new Canvas(mController.oWavePane.getBoundsInLocal().getWidth(), mController.oWavePane.getBoundsInLocal().getHeight());
        setupChWaveHandle(handleCh1, ch1Zero, Color.GREEN, -waveHandleWidth-2, axisXCh1);
        setupChWaveHandle(handleCh2, ch2Zero, Color.CYAN, 0, axisXCh2);
        mController.oWavePane.getChildren().addAll(wave, handleCh1, handleCh2, axisXCh1, axisXCh2);
        
        final double stageToWaveWindowOverhead = stage.getHeight() - mController.initWaveControl(mController.oWavePane, wave);
        stage.heightProperty().addListener((ObservableValue<? extends Number> observable, Number oldValue, Number newValue) -> {
            final double newWavePaneHeight = newValue.doubleValue() - stageToWaveWindowOverhead;
            mController.oWavePane.setPrefHeight(newWavePaneHeight);
        });
   }
    
    private void setupChWaveHandle(final Shape h, final DoubleProperty p, Paint fill, double offset, Line axisX) {
        p.setValue(0.);
        h.setFill(fill);
        h.setCursor(Cursor.V_RESIZE);
        h.translateXProperty().bind(Bindings.subtract(wave.widthProperty(), waveHandleWidth-offset));
        final NumberBinding y = Bindings.multiply(Bindings.add(p, 0.5), wave.heightProperty());
        h.translateYProperty().bind(Bindings.subtract(y, waveHandleHeight/2));
        h.setOnDragDetected((MouseEvent event) -> {
            if (draggingHandle != null || !(event.getSource() instanceof Shape))
                return;
            draggingHandle = event.getSource();
            event.consume();
        });
        h.setOnMouseDragged((MouseEvent event) -> {
            if (draggingHandle != event.getSource()) 
                return;
            double value = wave.sceneToLocal(0, event.getSceneY()).getY()/wave.getHeight()-0.5;
            if (value < -0.5)
                value = -0.5;
            else if (value > 0.5)
                value = 0.5;
            p.setValue(value);
            event.consume();
        });
        h.setOnMouseReleased((MouseEvent event) -> {
            if (draggingHandle == event.getSource())  {
                double value = wave.sceneToLocal(0, event.getSceneY()).getY()/wave.getHeight()-0.5;
                if (value < -0.5)
                    value = -0.5;
                else if (value > 0.5)
                    value = 0.5;
                p.setValue(value);
            }
            draggingHandle = null;
            event.consume();
        });
        axisX.setStroke(fill);
        axisX.getStrokeDashArray().addAll(5., 5.);
        axisX.setMouseTransparent(true);
        axisX.setStartX(0);
        axisX.endXProperty().bind(wave.widthProperty());
        axisX.startYProperty().bind(y);
        axisX.endYProperty().bind(y);
    }

    @Override
    public void updateWaveWindowBounds() {
        mController.waveWindowWidth = (int) mController.oWavePane.getBoundsInLocal().getWidth();
        Transform t = mController.oWavePane.getLocalToParentTransform();
        mController.waveWindowOrigin = t.transform(0, 0);
        t = mController.oWavePane.getLocalToSceneTransform();
        mController.waveWindowExtent = t.transform(mController.oWavePane.getWidth(), mController.oWavePane.getHeight());
        mController.crossHair2VLine.setVisible(false);
        mController.crossHair2HLine.setVisible(false);
        mController.crossHair2VLine.setStartY(mController.waveWindowOrigin.getY());
        mController.crossHair2VLine.setEndY(mController.waveWindowExtent.getY());
        mController.crossHair2HLine.setStartX(mController.waveWindowOrigin.getX());
        mController.crossHair2HLine.setEndX(mController.waveWindowExtent.getX());
        mController.timeScrollBar.setBlockIncrement(mController.waveWindowWidth);
        mController.updateWaveArrays();
    }
    

    
    @Override
    public void redrawWaves(int firstSampleIndex) {
        if (mController.zoomFactor < 3./65536)
            return;
        final int windowHeight = (int) wave.getHeight();
        final int windowWidth = mController.waveWindowWidth;
        final int totalSamples = dataRef.totalNumSamples;
        final int y0 = windowHeight / 32;
        final int y1 = windowHeight - y0;
        final double h = (y1 - y0) / 256.;
        final int sampleIndexToStart = firstSampleIndex > 0 ? firstSampleIndex - 1 : 0;
        final boolean ch1Enable = mController.ch1EnableToggleButton.isSelected();
        final boolean ch2Enable = mController.ch2EnableToggleButton.isSelected();
        if (totalSamples <= 0)
            return;
        Data.DataIterator d = dataIterator;
        if (d.init(sampleIndexToStart)) {
            final double xInc = 1/mController.zoomFactor;
            if (xInc > 10)
            {
                return;
            }
            double x = firstSampleIndex > 0 ? -xInc : 0;
            int xLast = -1;
            final GraphicsContext gc = wave.getGraphicsContext2D();
            gc.setFill(Color.BLACK);
            gc.fillRect(0, 0, windowWidth, windowHeight);
            gc.translate(0.5, 0.5);
            gc.setLineWidth(1);
            gc.setLineCap(StrokeLineCap.SQUARE);
            gc.setStroke(Color.rgb(0, 80, 40));
            gc.beginPath();
            final int gridStep = 100;
            for (int ix = 0; ix < windowWidth; ix += gridStep) {
                gc.moveTo(ix, 0);
                gc.lineTo(ix, windowHeight);
            }
            gc.moveTo(0, 0);
            gc.lineTo(windowWidth, 0);
            gc.stroke();
            gc.setLineWidth(1);
            do {
                    byte[] data1 = d.getData1();
                    byte[] data2 = d.getData2();
                    int o = d.getDataStart();
                    int dataLeft = d.getRemainingDataLength();
                    final double[] wavesX1 = mController.waves[0];
                    final double[] wavesY1 = mController.waves[1];
                    double[] wavesX2  = null;
                    final double[] wavesY2 = mController.waves[3];
                    int wavePoints1 = 0;
                    int wavePoints2 = 0;
                    if(d.getWidth1() > 10){
                        mController.widthText.setText(Double.toString(d.getWidth1())); 
                    }
                    if (xInc >= 1.) {
                        do {
                            wavesX1[wavePoints1] = x;
                            wavesY1[wavePoints1] = y1 - (data1[o]&0xff)*h;
                            wavesY2[wavePoints1] = y1 - (data2[o]&0xff)*h;
                            o++;
                            wavePoints1++;
                            x += xInc;
                        } while (x < (windowWidth+xInc) && wavePoints1 < wavesX1.length && --dataLeft > 0);
                        wavesX2 = wavesX1;
                        wavePoints2 = wavePoints1;
                    } else {
                        if (ch1Enable) {
                            wavePoints1 = drawCompressedWave(data1, o, wavesX1, wavesY1, xInc, dataLeft, x, y0, y1, h, xLast);
                            wavesY1[0] = wavesY1[1];
                        }
                        if (ch2Enable) {
                            wavesX2 = mController.waves[2];
                            wavePoints2 = drawCompressedWave(data2, o, wavesX2, wavesY2, xInc, dataLeft, x, y0, y1, h, xLast);
                        }
                    }
                    
                    if(wavePoints1 > 0){
                        xLast = (int)wavesX1[wavePoints1-1];
                    } else{
                        x = 0;
                    }
                    
                    if (xLast < mController.waveWindowWidth) {
                        x = xLast; 
                    } else {
                        x = 0;
                    }
                    
                    if ((ch1Enable) && (wavePoints1 > 0))  {
                        gc.setStroke(Color.GREEN);
                        final double ch1ZeroLevel = ch1Zero.doubleValue()*windowHeight;
                        gc.translate(0, ch1ZeroLevel);
                        gc.strokePolyline(wavesX1, wavesY1, wavePoints1);
                        gc.translate(0, -ch1ZeroLevel);
                    }
                    if (ch2Enable) {
                        gc.setStroke(Color.CYAN);
                        final double ch2ZeroLevel = ch2Zero.doubleValue()*windowHeight;
                        gc.translate(0, ch2ZeroLevel);
                        gc.strokePolyline(wavesX2, wavesY2, wavePoints2);
                        gc.translate(0, -ch2ZeroLevel);
                    }
         
                    d.skipData();
            } while (x <= windowWidth && d.hasMoreData());
            gc.translate(-0.5, -0.5);
        }        
        mController.timeValueText.setText(dataRef.sampleToTime(firstSampleIndex, mController.timeUnits));
    }
    int min = 0;
    int max = 0;
    int cur = 0;
    private int drawCompressedWave(byte[] data, int o, double[] wavesX, double[] wavesY, 
            double xInc, int dataLeft, double x, int y0, int y1, double h, int xLast) {
        int wavePoints = 0;
        int xCur = (int)x;
        xLast = xCur;
        int k=0;
        do {
            if (xCur == xLast) {
                cur += data[o++] & 0xff;
                dataLeft--;    
                k++;
            } else {
                wavesX[wavePoints] = xLast;
                wavesY[wavePoints++] = y1 - (cur/k)*h;
                k = 1;
                cur = data[o++] & 0xff;
                dataLeft--;
                xLast = xCur;
            }
            x += xInc;
            xCur=(int)x;
        } while (x <= mController.waveWindowWidth && wavePoints < wavesX.length && dataLeft > 0);
        
        if (wavePoints == 0) 
        {
            wavesX[wavePoints] = xCur;
            wavesY[wavePoints++] = y1 - (cur/k)*h;
        }
        
        
        return wavePoints;
    }

    @Override
    public void updateMouseInfo(int localX, int sceneX, int localY, int mousePos) {
        final String posStr = dataRef.sampleToTime((int)(mousePos*mController.zoomFactor), mController.timeUnits);
        final String relPosStr = dataRef.sampleToTime((int)(localX*mController.zoomFactor), mController.timeUnits);
        if (posStr.length() > 0)
            mController.cursorInfoLabel2.setText(String.format("at %s %s (+%s %s)", 
                    posStr, 
                    MainController.timeUnitsText[mController.timeUnits],
                    relPosStr,
                    MainController.timeUnitsText[mController.timeUnits]));
        mController.crossHair2VLine.setStartX(sceneX);
        mController.crossHair2VLine.setEndX(sceneX);
        mController.crossHair2VLine.setVisible(true);
        mController.crossHair2HLine.setStartY(localY);
        mController.crossHair2HLine.setEndY(localY);
        mController.crossHair2HLine.setVisible(true);
    }
    
    @Override
    public void dataLoaded(boolean modeActive) {
        dataRef = Main.oData;
        dataIterator = dataRef.new DataIterator();
        mController.capturedInfoLabel2.setText("last capture at "+dataRef.getFrequency().getText());
    }
    
    @Override
    public Data getData() {
        return dataRef;
    }
}
