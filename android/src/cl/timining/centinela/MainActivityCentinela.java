package cl.timining.centinela;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Environment;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

public class MainActivityCentinela extends Activity {
	private static final String TAG = "MainActivityCentinela";

	private final ExecutorService exec = Executors.newSingleThreadExecutor();

	private UsbManager manager;
	public UsbSerialDriver device;
	private SerialInputOutputManager ioManager;
	private SerialInputOutputManager.Listener ioListener = new SerialInputOutputManager.Listener() {
		@Override
		public void onNewData(final byte[] data) {
			MainActivityCentinela.this.runOnUiThread(new Runnable() {
				@Override
				public void run() {
					MainActivityCentinela.updateDataIn(MainActivityCentinela.this, data);
				}
			});
		}

		@Override
		public void onRunError(Exception arg0) {
			// TODO Auto-generated method stub			
		}	
	};

	public DBHelper dbhelper;
	static public final File dir = new File(Environment.getExternalStorageDirectory(), "Centinela");
	private String filename;
	private FileOutputStream outputStream;
	private int dataCount = 0;

	private TextView log;
	static public int baudrate = 115200;
	static public int timeout = 1000;
	static public int log_cmd = 0;

	/*@Override
	protected void onStart() {
		// TODO Auto-generated method stub
		super.onStart();
		
		Intent intent = new Intent(this, GraphActivity.class);
        intent.putExtra("filename", dir + "/" + "CENT000.BIN");
        
    	startActivity(intent);
	}*/
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main_activity_centinela);

		// Get UsbManager from Android.
        manager = (UsbManager) getSystemService(Context.USB_SERVICE);

        log = (TextView)this.findViewById(R.id.log);
        log.setTextColor(Color.parseColor("#e1bd16"));
        log.setBackgroundColor(Color.parseColor("#38c175"));
        log.setMovementMethod(new ScrollingMovementMethod());

        Button start = (Button)this.findViewById(R.id.start);
        start.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
    			sendCmd(new byte[] { 'n' });
            }
        });

        Button ls = (Button)this.findViewById(R.id.ls);
        ls.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
    			sendCmd(new byte[] { 'l' });
            }
        });

        if (!dir.exists()) dir.mkdirs();

        dbhelper = new DBHelper(this);

        FileCursorAdapter adapter = new FileCursorAdapter(this, dbhelper.getFilesCursor());

		ListView listView = (ListView) findViewById(R.id.listFiles);
		listView.setAdapter(adapter);
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
				device.setBaudRate(baudrate);
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

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main_activity_centinela, menu);
		return true;
	}

	@Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
        case (R.id.action_settings):
            Intent settings = new Intent(this, SettingsActivity.class);
        /*
            settings.putExtra("url", dbHelper.getURL());
            settings.putExtra("auth", dbHelper.getAuth());
            settings.putExtra("username", dbHelper.getUsername());
            settings.putExtra("password", dbHelper.getPassword());*/
            startActivityForResult(settings, 0);
            return true;
        }
        return super.onOptionsItemSelected(item);
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

	private void sendCmd(byte[] cmd) {
		Log.v(TAG, "device:"+device+":cmd:"+new String(cmd));
		if (device == null) return;

		try {
			device.write(cmd, timeout);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	static public void updateDataIn(Context context, byte[] data) {
		int head_length = 9;
		if (data.length > head_length) {
			String head = "head:";
			for (int i = 0; i <= head_length; i++)
				head += data[i] + ":";
			Log.v(TAG, head+data.length);
		}

		int ini = 0;
		int end = indexOf(data, ini, (byte)-86);
		while (end != -1 && data.length > end+head_length) {
			if (data[end+1] != (byte)-86 ||
				data[end+2] != (byte)-86 ||
				data[end+3] != (byte)-1  ||
				data[end+4] != (byte)-1  ||
				data[end+5] != (byte)-1  ||
				data[end+6] != (byte) 0  ||
				data[end+7] != (byte) 0  ||
				data[end+8] != (byte) 0) {
				end = indexOf(data, end+1, (byte)-86);
				continue;
			}
			Log.v(TAG, "ini:"+ini);
			Log.v(TAG, "end:"+end);
			
			if (end - ini > 0) {
				byte[] subData = new byte[end-ini];
				for (int i = 0; i < end-ini; i++) subData[i] = data[ini+i];
				processSubData(context, subData);
			}

			if (data[end+head_length] == (byte)116) log_cmd = 1;
			if (data[end+head_length] == (byte)108) log_cmd = 2;
			if (data[end+head_length] == (byte)102) log_cmd = 3;
			if (data[end+head_length] == (byte)103) log_cmd = 4;
			if (data[end+head_length] == (byte)101) log_cmd = 5;
			if (data[end+head_length] == (byte)115) log_cmd = 6;
			Log.v(TAG, "log_cmd:"+log_cmd);

			ini = end+head_length+1;
			end = indexOf(data, ini, (byte)-86);
		}

		if (data.length - ini > 0) {
			Log.v(TAG, "tail...");
			byte[] subData = new byte[data.length-ini];
			for (int i = 0; i < data.length-ini; i++) subData[i] = data[ini+i];
			processSubData(context, subData);
		}
	}

	private static void processSubData(Context context, byte[] data)
	{
		Log.v(TAG, "log_cmd:"+log_cmd);
		Log.v(TAG, "data.size:"+data.length);

		FileCursorAdapter adapter;
		ListView listView;

		MainActivityCentinela mc = null;
		SettingsActivity sa = null;
		if (context.getClass()==MainActivityCentinela.class) {
			mc = (MainActivityCentinela)context;
		} else if (context.getClass()==SettingsActivity.class) {
			sa = (SettingsActivity)context;
		}

		switch (log_cmd) {
		case 1: // log
			String message = new String (data);
			Log.v(TAG, "data:"+message);

			if (mc!=null) mc.appendLog(message);
			if (sa!=null) sa.appendLog(message);

			break;
		case 2:  // ls
			if (mc!=null) {
				String filelist = new String (data);
				filelist = filelist.replace("\r", "");
				filelist = filelist.replace("CNT.CFG\n", ""); // remove configuration file!
				if (filelist.length() > 0) {
					String[] files = filelist.split("\n");
					mc.dbhelper.addFiles(files);
				}
				adapter = new FileCursorAdapter(context, mc.dbhelper.getFilesCursor());

				listView = (ListView) mc.findViewById(R.id.listFiles);
				listView.setAdapter(adapter);
			}
			break;
		case 3: // file incoming
			if (mc!=null) {
				if (mc.outputStream == null) {
					int n = data[0];
					if (data.length > n) {
		  				byte[] subdata = new byte[n];
						for (int i = 0; i < n; i++) subdata[i] = data[i+1];
						mc.filename = new String(subdata);
						File file = new File(dir, mc.filename);
						try {
							mc.outputStream = new FileOutputStream(file);
							byte[] tail = new byte[data.length - (n+1)];
							for (int i = 0; i < tail.length; i++) tail[i] = data[n+1+i];
							mc.outputStream.write(tail);
						} catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
							mc.outputStream = null;
						}

						mc.appendLog("open:"+mc.filename+" ");
					}
				} else {
					try {
						mc.outputStream.write(data);
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}

					if (mc.dataCount%10 == 0) mc.appendLog(".");
					mc.dataCount++;
				}
			}
			break;
		case 4: // file end
			if (mc!=null) {
				if (mc.outputStream != null) {
					//log.append("close:"+file.getName()+"\n");
					try {
						mc.outputStream.close();
						mc.appendLog(" close:"+mc.filename+"\n");
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}

					if (mc.dbhelper.updateStatus(mc.filename, 1) > 0) {
				        adapter = new FileCursorAdapter(context, mc.dbhelper.getFilesCursor());
						listView = (ListView) mc.findViewById(R.id.listFiles);
						listView.setAdapter(adapter);

						mc.appendLog("update:"+mc.filename+":"+mc.filename.length()+"\n");
					}

					mc.appendLog("status:"+mc.dbhelper.getStatus(mc.filename)+"\n");

					mc.filename = null;
					mc.outputStream = null;
				}
			}
			break;
		case 5: // end of datalogging
			if (mc!=null) mc.sendCmd(new byte[]{ 'l' });
			if(sa!=null) sa.sendCmd(new byte[]{ 'l' });
			break;
		case 6:
			if (mc!=null) mc.appendLog("O_o\n");
			if (sa!=null) {
				if (data.length >=13) {
					for (int i = 0; i < data.length; i++) Log.v(TAG, "data["+i+"]:"+(int)(data[i] & 0xff));

					int nch = (data[0] & 0xff);

					long tick_time_usec = 0;
					long time_max_msec = 0;
					for (int i=0; i < 4; i++) {
						tick_time_usec = (tick_time_usec << 8) + (data[4-i] & 0xff);
						time_max_msec = (time_max_msec << 8) + (data[8-i] & 0xff);
					}

					int adc_buffer_size = 0;
					int sd_buffer_size = 0;
					for (int i=0; i < 2; i++) {
						adc_buffer_size = (adc_buffer_size << 8) + (data[10-i] & 0xff);
						sd_buffer_size = (sd_buffer_size << 8) + (data[12-i] & 0xff);
					}

					sa.refreshGUI(nch, tick_time_usec, time_max_msec);
				}
			}
			break;
		}
	}
	
	private static int indexOf(byte[] data, int ini, byte b)
	{
		for (int i = ini; i < data.length; i++) {
			if (data[i] == b) return i;
		}
		return -1;
	}

	private void appendLog(String message)
	{
		if (log !=null) {
			log.append(message);

			int scrollAmount = log.getLayout().getLineTop(log.getLineCount());
			scrollAmount -= log.getHeight();

			if (scrollAmount < 0) scrollAmount = 0;
			log.scrollTo(0, scrollAmount);
		}
	}
}