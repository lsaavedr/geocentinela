package cl.timining.geocentinela;

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
	// private static final String TAG = "GraphActivity";

	private String filename;
	private MultitouchPlot mySimpleXYPlot;
	private List<Number>[] numbers;

	private int head_size = 26;
	private int tail_size = 12;
	private static int BUFFER_SIZE = 6000;

	private double Vcte = 1200/Math.pow(2, 16);
	private double Voffset = 600;

	// head
	private int nch = 3;
	private int average = 0;
	private long tick_time_useg = 1;
	private int gain = 1;
	private int time_type = 0;
	private long time_begin_seg = 0;
	private long time_end_seg = 0;
	private long rtc_begin = 0;
	private long adc_play_cnt = 0;
	private long adc_error_cfg = 0;

	private double Tcte = 0.125;
	private double Tmin = 0;
	private double Tmax = 20000;

	private int med = 20;
	
	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_graph);
		this.filename = getIntent().getStringExtra("filename");
		File file = new File(this.filename);

		DataInputStream dis;
		try {
			dis = new DataInputStream(new FileInputStream(file));

			byte[] format = new byte[1];
			byte[] head = new byte[head_size];
			byte[] tail = new byte[tail_size];
			byte[] buf = new byte[BUFFER_SIZE];

			int nread = dis.read(format);

			nch = ((format[0] & 0xff) >> 4) & 0xf;
			int version = 0;
			if (nch==0) {
				version = format[0] & 0xff;
				nch = 3;
			}

			nread = dis.read(head);
			if (nread == head_size) {
				switch (version) {
				case 0: {
					nch = ((format[0] & 0xff) >> 4) & 0xf;
					average = (format[0] & 0xff) & 0xf;

					gain = (head[4] & 0xff);
					time_type = (head[5] & 0xff);

					tick_time_useg = 0;
					time_begin_seg = 0;
					time_end_seg = 0;
					rtc_begin = 0;
					adc_play_cnt = 0;
					adc_error_cfg = 0;

					for (int i=0; i < 4; i++) {
						tick_time_useg += (tick_time_useg << 8) + (head[3-i] & 0xff);
						time_begin_seg += (time_begin_seg << 8) + (head[9-i] & 0xff);
						time_end_seg += (time_end_seg << 8) + (head[13-i] & 0xff);
						rtc_begin += (rtc_begin << 8) + (head[17-i] & 0xff);
						adc_play_cnt += (adc_play_cnt << 8) + (head[21-i] & 0xff);
						adc_error_cfg += (adc_error_cfg << 8) + (head[25-i] & 0xff);
					}
				} break;
				case 1: {
					nch = 3;

					gain = ((head[0] & 0xff) >> 4) & 0xf;
					average = (head[0] & 0xff) & 0xf;

					time_type = (head[5] & 0xff);

					tick_time_useg = 0;
					time_begin_seg = 0;
					time_end_seg = 0;
					rtc_begin = 0;
					adc_play_cnt = 0;
					adc_error_cfg = 0;

					for (int i=0; i < 4; i++) {
						tick_time_useg += (tick_time_useg << 8) + (head[4-i] & 0xff);
						time_begin_seg += (time_begin_seg << 8) + (head[9-i] & 0xff);
						time_end_seg += (time_end_seg << 8) + (head[13-i] & 0xff);
						rtc_begin += (rtc_begin << 8) + (head[17-i] & 0xff);
						adc_play_cnt += (adc_play_cnt << 8) + (head[21-i] & 0xff);
						adc_error_cfg += (adc_error_cfg << 8) + (head[25-i] & 0xff);
					}
				} break;
				}

				Tcte = tick_time_useg/1000.0;
			}

			numbers = (List<Number>[])new List[nch];
			for (int i = 0; i < nch; i++) numbers[i] = new Vector<Number>();

			int[] sample = new int[nch];
			double[] sumSample = new double[nch];

			int count = 0;
			while ((nread = dis.read(buf)) >= 0) {
				for (int i = 0; i < nread; i += 2*nch) {
					for (int j = 0; j < nch; j++) {
						sample[j] = ((buf[i + 2*j + 1] & 0xff) << 8) + (buf[i + 2*j] & 0xff);
						sumSample[j] += Vcte*sample[j]-Voffset;

						if (count%med == med-1) {
							numbers[j].add(sumSample[j]/20.0);
							sumSample[j] = 0;
						}
					}
					count++;
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
		mySimpleXYPlot.setTitle("test view");
		mySimpleXYPlot.setRangeLabel("mV");
		mySimpleXYPlot.setDomainLabel("s");

		mySimpleXYPlot.setDomainStep(XYStepMode.INCREMENT_BY_VAL, 100);
		mySimpleXYPlot.setDomainValueFormat(new NumberFormat() {
			/**
			 * 
			 */
			private static final long serialVersionUID = 1234567890L;

			@Override
            public StringBuffer format(double d, StringBuffer sb, FieldPosition fp)
			{
                return sb.append(String.format("%,2.3f", (Tmin + d * Tcte)/1000.0)); // shortcut to convert d+1 into a String
            }

            // unused
            @Override
            public StringBuffer format(long l, StringBuffer stringBuffer, FieldPosition fieldPosition) { return null; }

            // unused
            @Override
            public Number parse(String s, ParsePosition parsePosition) { return null; }
        });
	}
}