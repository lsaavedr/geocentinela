package cl.timining.centinela;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.app.Activity;
import android.app.Dialog;
import android.content.Context;
import android.graphics.Color;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.NumberPicker;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

public class SettingsActivity extends Activity {
	private static final String TAG = "SettingsActivity";

	private final ExecutorService exec = Executors.newSingleThreadExecutor();

	private UsbManager manager;
	private UsbSerialDriver device;
	private SerialInputOutputManager ioManager;
	private SerialInputOutputManager.Listener ioListener = new SerialInputOutputManager.Listener() {
		@Override
		public void onNewData(final byte[] data) {
			SettingsActivity.this.runOnUiThread(new Runnable() {
				@Override
				public void run() {
					MainActivityCentinela.updateDataIn(SettingsActivity.this, data);
				}
			});
		}

		@Override
		public void onRunError(Exception arg0) {
			// TODO Auto-generated method stub
		}
	};

	private TextView log;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main_settings_centinela);

		// Get UsbManager from Android.
		manager = (UsbManager) getSystemService(Context.USB_SERVICE);

		final TextView nch = (TextView)this.findViewById(R.id.nch);
		nch.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				final Dialog dialog = new Dialog(SettingsActivity.this);
				dialog.setContentView(R.layout.number_picker);

				final NumberPicker np = (NumberPicker) dialog.findViewById(R.id.np);
				np.setMinValue(1);
				np.setMaxValue(9);

				Button ok = (Button)dialog.findViewById(R.id.ok);
				ok.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	nch.setText(""+np.getValue());
		            	dialog.dismiss();
		            }
		        });

				dialog.show();
			}
		});

		final TextView sps = (TextView)this.findViewById(R.id.sps);
		sps.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				final Dialog dialog = new Dialog(SettingsActivity.this);
				dialog.setContentView(R.layout.number_picker);

				final NumberPicker np = (NumberPicker) dialog.findViewById(R.id.np);
				final String[] values = new String[]{ "2000", "4000", "6000", "8000", "10000" };
				np.setMinValue(0);
				np.setMaxValue(values.length-1);
				np.setDisplayedValues(values);

				Button ok = (Button)dialog.findViewById(R.id.ok);
				ok.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	sps.setText(""+values[np.getValue()]);
		            	dialog.dismiss();
		            }
		        });

				dialog.show();
			}
		});

		final TextView dtime = (TextView)this.findViewById(R.id.dtime);
		dtime.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				final Dialog dialog = new Dialog(SettingsActivity.this);
				dialog.setContentView(R.layout.number_picker);

				final NumberPicker np = (NumberPicker) dialog.findViewById(R.id.np);
				final String[] values = new String[]{ "1000", "5000", "10000", "15000", "20000"};
				np.setMinValue(0);
				np.setMaxValue(values.length-1);
				np.setDisplayedValues(values);

				Button ok = (Button)dialog.findViewById(R.id.ok);
				ok.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	dtime.setText(""+values[np.getValue()]);
		            	dialog.dismiss();
		            }
		        });

				dialog.show();
			}
		});

		Button save = (Button)this.findViewById(R.id.save);
		save.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	String str_nch = nch.getText().toString();
            	String str_sps = sps.getText().toString();
            	String str_dtime = dtime.getText().toString();

            	if (str_nch.length() > 0) {
            		int int_nch = Integer.parseInt(str_nch);
            		Log.v(TAG, "nch:"+(byte)(int_nch & 0xff));
            		sendCmd(new byte[] { 's', 'n', (byte)(int_nch & 0xff) });
            	}

            	if (str_sps.length() > 0) {
            		double dbl_sps = Double.parseDouble(str_sps);
            		long usec = (long)(1000000.0/dbl_sps);

            		Log.v(TAG, "sps:"+(byte)(usec & 0xff)+
            				":"+(byte)((usec >> 8) & 0xff)+
            				":"+(byte)((usec >> 16) & 0xff)+
            				":"+(byte)((usec >> 32) & 0xff));

            		sendCmd(new byte[] { 's', 's',
            				(byte)(usec & 0xff),
            				(byte)((usec >> 8) & 0xff),
            				(byte)((usec >> 16) & 0xff),
            				(byte)((usec >> 32) & 0xff)});
            	}

            	if (str_dtime.length() > 0) {
            		long long_dtime = Long.parseLong(str_dtime);

            		Log.v(TAG, "dtime:"+(byte)(long_dtime & 0xff)+
            				":"+(byte)((long_dtime >> 8) & 0xff)+
            				":"+(byte)((long_dtime >> 16) & 0xff)+
            				":"+(byte)((long_dtime >> 32) & 0xff));

            		sendCmd(new byte[] { 's', 't',
            				(byte)(long_dtime & 0xff),
            				(byte)((long_dtime >> 8) & 0xff),
            				(byte)((long_dtime >> 16) & 0xff),
            				(byte)((long_dtime >> 32) & 0xff)});
            	}

            	finish();
            }
        });

		Button refresh = (Button)this.findViewById(R.id.refresh);
		refresh.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
    			sendCmd(new byte[] { 's', 'g' });
            }
        });

		log = (TextView)this.findViewById(R.id.log);
		log.setTextColor(Color.parseColor("#e1bd16"));
		log.setBackgroundColor(Color.parseColor("#38c175"));
		log.setMovementMethod(new ScrollingMovementMethod());
	}

	private void startIOManager() {
		if (device != null) {
			ioManager = new SerialInputOutputManager(device, ioListener);
			exec.submit(ioManager);
		}
	}

	private void stopIOManager () {
		if (ioManager != null) {
			ioManager.stop();
			ioManager = null;
		}
	}

	private void restartIOManager() {
		stopIOManager();
		startIOManager();
	}

	@Override
	protected void onPause() {
		super.onPause();
		stopIOManager();

		if (device != null) {
			try {
				device.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			device = null;
		}
	}

	@Override
	protected void onResume() {
		super.onResume();

		device = UsbSerialProber.acquire(manager);
		if (device == null) {
			log.setText("No serial device.");
		} else {
			try {
				device.open();
				device.setBaudRate(MainActivityCentinela.baudrate);
			} catch (IOException e) {
				e.printStackTrace();
				log.setText("Error opening device: " + e.getMessage());

				try {
					device.close();
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				device = null;
			}
			log.setText("Serial device: " + device + "\n");
		}

		sendCmd(new byte[] { 's', 'g' });
		restartIOManager();
	}

	@Override
	protected void onDestroy()
	{
		if (device!=null) {
			try {
				device.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    }
		super.onDestroy();
	}

	public void sendCmd(byte[] cmd) {
		Log.v(TAG, "device:"+device+":cmd:"+new String(cmd));
		if (device == null) return;

		try {
			device.write(cmd, MainActivityCentinela.timeout);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void appendLog(String message)
	{
		if (log !=null) {
			log.append(message);

			int scrollAmount = log.getLayout().getLineTop(log.getLineCount());
			scrollAmount -= log.getHeight();

			if (scrollAmount < 0) scrollAmount = 0;
			log.scrollTo(0, scrollAmount);
		}
	}

	public void refreshGUI(int nch, long tick_time_usec, long time_max_msec)
	{
		TextView nchview = (TextView)this.findViewById(R.id.nch);
		TextView sps = (TextView)this.findViewById(R.id.sps);
		TextView dtime = (TextView)this.findViewById(R.id.dtime);

		nchview.setText(""+nch);
		sps.setText(""+(1000000.0/tick_time_usec));
		dtime.setText(""+time_max_msec);
	}
}