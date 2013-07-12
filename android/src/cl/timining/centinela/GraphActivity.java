package cl.timining.centinela;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.text.ParsePosition;
import java.util.List;
import java.util.Vector;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;

import com.androidplot.series.XYSeries;
import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYStepMode;

public class GraphActivity extends Activity
{
	private String filename;
	private MultitouchPlot mySimpleXYPlot;
	private static int BUFFER_SIZE = 6000;
	private List<Number>[] numbers;

	private double Vcte = 1198/Math.pow(2, 16);
	private double Voffset = 601;

	private int nch = 3;

	private double Tcte = 0.000125;
	private double Tmin = 0;
	private double Tmax = 20000;

	private long denominator = 50;
	private long cntTime = 0;

	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_graph);
		this.filename = getIntent().getStringExtra("filename");
		File file = new File(this.filename);

		/*
		numbers0 = new Vector<Number>();
		numbers1 = new Vector<Number>();
		numbers2 = new Vector<Number>();
		*/

		DataInputStream dis;
		try {
			dis = new DataInputStream(new FileInputStream(file));
			byte[] head = new byte[13];
			byte[] buf = new byte[BUFFER_SIZE];

			int nread = dis.read(head);
			if (nread == 13) {
				nch = (head[0] & 0xff);

				long tick_time_usec = 0;
				long time_max_msec = 0;
				long rtc_time = 0;
				for (int i=0; i < 4; i++) {
					tick_time_usec = (tick_time_usec << 8) + (head[4-i] & 0xff);
					time_max_msec = (time_max_msec << 8) + (head[8-i] & 0xff);
					rtc_time = (rtc_time << 8) + (head[12-i] & 0xff);
				}

				Tcte = tick_time_usec/1000000.0;
				Tmax = time_max_msec;
			}

			numbers = (List<Number>[])new List[nch];
			for (int i = 0; i < nch; i++) numbers[i] = new Vector<Number>();

			int[] sample = new int[nch];
			double[] sumSample = new double[nch];

			int counterSamples = 0;

			while ((nread = dis.read(buf)) >= 0 && cntTime < Tmax) {
				for (int i = 0; i < nread; i += 2*nch) {
					counterSamples++;

					for (int j = 0; j < nch; j++) {
						sample[j] = ((buf[i + 2*j + 1] & 0xff) << 8) + (buf[i + 2*j] & 0xff);
						sumSample[j] += Vcte*sample[j]-Voffset;

						if (counterSamples >= denominator) {
							sumSample[j] /= counterSamples;
							numbers[j].add(sumSample[j]);

							sumSample[j] = 0;
						}
					}

					if (counterSamples >= denominator) {
						counterSamples = 0;
						cntTime += Tcte*denominator;
						if (cntTime > Tmax) break;
					}
				}
			}
			dis.close();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	
		mySimpleXYPlot = (MultitouchPlot) findViewById(R.id.mySimpleXYPlot);
		int[] paletaColor = new int[]{ 255,   0,   0,
				                          0, 255,   0,
				                          0,   0, 255,
				                        128, 128,   0,
				                          0, 128, 128,
				                        128,   0, 128,
				                         64, 128,  64,
				                         64,  64, 128,
				                        128,  64,  64};
		

		for (int i = 0; i < nch; i++) {
			// Turn the above arrays into XYSeries':
			XYSeries series = new SimpleXYSeries(numbers[i], // SimpleXYSeries

					SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, // Y_VALS_ONLY means use
															// the element index as
															// the x value
					"Ch"+i);
			LineAndPointFormatter seriesFormat = new LineAndPointFormatter(
					Color.rgb(paletaColor[3*i], paletaColor[3*i+1], paletaColor[3*i+2]), null, null);

			// add a new series' to the xyplot:
			mySimpleXYPlot.addSeries(series, seriesFormat);
		}

		mySimpleXYPlot.calculateMinMaxVals();
		mySimpleXYPlot.setBackgroundColor(Color.TRANSPARENT);
		mySimpleXYPlot.setTitle("teensy 3.0 at 8ksps");
		mySimpleXYPlot.setRangeLabel("mV");
		mySimpleXYPlot.setDomainLabel("s");

		mySimpleXYPlot.setDomainStep(XYStepMode.INCREMENT_BY_VAL, denominator);
		mySimpleXYPlot.setDomainValueFormat(new NumberFormat() {
			@Override
            public StringBuffer format(double d, StringBuffer sb, FieldPosition fp) {
                return sb.append(String.format("%,2.3f", d * Tcte * denominator)); // shortcut to convert d+1 into a String
            }

            // unused
            @Override
            public StringBuffer format(long l, StringBuffer stringBuffer, FieldPosition fieldPosition) { return null;}

            // unused
            @Override
            public Number parse(String s, ParsePosition parsePosition) { return null;}
        });
	}
}