package cl.timining.centinela;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.TimePickerDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.Color;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.NumberPicker;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.TimePicker;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

public class SettingsActivity extends Activity {
	//private static final String TAG = "SettingsActivity";

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

    /*
     * 2Formetter
     */
    public static final NumberPicker.Formatter TWO_DIGIT_FORMATTER =
    new NumberPicker.Formatter() {
    	final StringBuilder mBuilder = new StringBuilder();
        final java.util.Formatter mFmt = new java.util.Formatter(mBuilder);
        final Object[] mArgs = new Object[1];

        public String toString(int value) {
            mArgs[0] = value;
            mBuilder.delete(0, mBuilder.length());
            mFmt.format("%02d", mArgs);
            return mFmt.toString();
        }

		@Override
		public String format(int value) {
			// TODO Auto-generated method stub
			return this.toString(value);
		}
    };

    private static int[] getTimeNumbers(String time) {
    	String[] data = time.split(":");
    	int[] num = new int[3];

    	num[0] = 0;
		try {
			num[0] = Integer.parseInt(data[0]);
		} catch (Exception e) {
			num[0] = 0;
		}

		num[1] = 0;
		try {
			num[1] = Integer.parseInt(data[1]);
		} catch (Exception e) {
			num[1] = 0;
		}

		num[2] = 0;
		try {
			num[2] = Integer.parseInt(data[2]);
		} catch (Exception e) {
			num[2] = 0;
		}

		return num;
    }

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

				Button cancel = (Button)dialog.findViewById(R.id.cancel);
				cancel.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
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
				np.setMinValue(1);
				np.setMaxValue(10);

				Button ok = (Button)dialog.findViewById(R.id.ok);
				ok.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	sps.setText(""+np.getValue());
		            	dialog.dismiss();
		            }
		        });

				Button cancel = (Button)dialog.findViewById(R.id.cancel);
				cancel.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	dialog.dismiss();
		            }
		        });

				dialog.show();
			}
		});

		final TextView haverage = (TextView)this.findViewById(R.id.haverage);
		haverage.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				final Dialog dialog = new Dialog(SettingsActivity.this);
				dialog.setContentView(R.layout.number_picker);

				final NumberPicker np = (NumberPicker) dialog.findViewById(R.id.np);
				final String[] values = new String[]{ "0", "4", "8", "16", "32"};
				np.setMinValue(0);
				np.setMaxValue(values.length-1);
				np.setDisplayedValues(values);

				Button ok = (Button)dialog.findViewById(R.id.ok);
				ok.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	haverage.setText(""+values[np.getValue()]);
		            	dialog.dismiss();
		            }
		        });

				Button cancel = (Button)dialog.findViewById(R.id.cancel);
				cancel.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
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
				final String[] values = new String[]{ "5", "10", "15", "20"};
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

				Button cancel = (Button)dialog.findViewById(R.id.cancel);
				cancel.setOnClickListener(new View.OnClickListener() {
		            @Override
		            public void onClick(View v) {
		            	dialog.dismiss();
		            }
		        });

				dialog.show();
			}
		});

		final CheckBox chrono = (CheckBox)this.findViewById(R.id.chrono);
		final RelativeLayout chrono_layout = (RelativeLayout)this.findViewById(R.id.chrono_layout);

		final CheckBox daily = (CheckBox)this.findViewById(R.id.daily);
		final RelativeLayout daily_layout = (RelativeLayout)this.findViewById(R.id.daily_layout);

		chrono.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				daily.setChecked(false);
				daily_layout.setEnabled(false);
				daily_layout.setVisibility(View.INVISIBLE);
				chrono_layout.setVisibility(View.VISIBLE);
				chrono_layout.setEnabled(true);
				chrono.setChecked(true);
			}
		});
		daily.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View arg0) {
				chrono.setChecked(false);
				chrono_layout.setEnabled(false);
				chrono_layout.setVisibility(View.INVISIBLE);
				daily_layout.setVisibility(View.VISIBLE);
				daily_layout.setEnabled(true);
				daily.setChecked(true);
			}
		});

		final TextView delay = (TextView)this.findViewById(R.id.delay);
		delay.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (!chrono.isChecked()) chrono.performClick();

				// get time
				int[] time = getTimeNumbers(delay.getText().toString());

				// widget
				View view = SettingsActivity.this.getLayoutInflater().inflate(R.layout.time_picker, null);

				AlertDialog.Builder alert = new AlertDialog.Builder(SettingsActivity.this);
				alert.setView(view);
				alert.create();
				alert.setTitle("Pick a delay");

				final NumberPicker hour = (NumberPicker) view.findViewById(R.id.hour);
				hour.setMinValue(0);
				hour.setMaxValue(23);
				hour.setValue(time[0]);
				hour.setFormatter(TWO_DIGIT_FORMATTER);

				final NumberPicker minute = (NumberPicker) view.findViewById(R.id.minute);
				minute.setMinValue(0);
				minute.setMaxValue(59);
				minute.setValue(time[1]);
				minute.setFormatter(TWO_DIGIT_FORMATTER);

				final NumberPicker second = (NumberPicker) view.findViewById(R.id.second);
				second.setMinValue(0);
				second.setMaxValue(59);
				second.setValue(time[2]);
				second.setFormatter(TWO_DIGIT_FORMATTER);

				alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                    	String hour_str = ""+hour.getValue();
                    	if (hour.getValue() < 10) hour_str = "0"+hour.getValue();

                    	String minute_str = ""+minute.getValue();
                    	if (minute.getValue() < 10) minute_str = "0"+minute.getValue();

                    	String second_str = ""+second.getValue();
                    	if (second.getValue() < 10) second_str = "0"+second.getValue();

                    	delay.setText(hour_str+":"+minute_str+":"+second_str);
                    }
                });

				alert.show();
			}
		});

		final TextView lapse = (TextView)this.findViewById(R.id.lapse);
		lapse.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (!chrono.isChecked()) chrono.performClick();

				// get time
				int[] time = getTimeNumbers(lapse.getText().toString());

				// widget
				View view = SettingsActivity.this.getLayoutInflater().inflate(R.layout.time_picker, null);

				AlertDialog.Builder alert = new AlertDialog.Builder(SettingsActivity.this);
				alert.setView(view);
				alert.create();
				alert.setTitle("Pick a lapse");
				
				final NumberPicker hour = (NumberPicker) view.findViewById(R.id.hour);
				hour.setMinValue(0);
				hour.setMaxValue(23);
				hour.setValue(time[0]);
				hour.setFormatter(TWO_DIGIT_FORMATTER);

				final NumberPicker minute = (NumberPicker) view.findViewById(R.id.minute);
				minute.setMinValue(0);
				minute.setMaxValue(59);
				minute.setValue(time[1]);
				minute.setFormatter(TWO_DIGIT_FORMATTER);

				final NumberPicker second = (NumberPicker) view.findViewById(R.id.second);
				second.setMinValue(0);
				second.setMaxValue(59);
				second.setValue(time[2]);
				second.setFormatter(TWO_DIGIT_FORMATTER);

				alert.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                    	String hour_str = ""+hour.getValue();
                    	if (hour.getValue() < 10) hour_str = "0"+hour.getValue();

                    	String minute_str = ""+minute.getValue();
                    	if (minute.getValue() < 10) minute_str = "0"+minute.getValue();

                    	String second_str = ""+second.getValue();
                    	if (second.getValue() < 10) second_str = "0"+second.getValue();

                    	lapse.setText(hour_str+":"+minute_str+":"+second_str);
                    }
                });

				alert.show();
			}
		});

		final TextView begin = (TextView)this.findViewById(R.id.begin);
		begin.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (!daily.isChecked()) daily.performClick();

				// get time
				int[] time = getTimeNumbers(begin.getText().toString());

				TimePickerDialog tp = new TimePickerDialog(SettingsActivity.this, new TimePickerDialog.OnTimeSetListener() {
					@Override
					public void onTimeSet(TimePicker view, int hourOfDay, int minute) {
						// TODO Auto-generated method stub
						if (minute > 9 && hourOfDay > 9)
							begin.setText(""+hourOfDay+":"+minute);
						else if (minute < 9 && hourOfDay < 9)
							begin.setText("0"+hourOfDay+":0"+minute);
						else if (minute > 9)
							begin.setText("0"+hourOfDay+":"+minute);
						else
							begin.setText(""+hourOfDay+":0"+minute);
					}
				}, time[0], time[1], true);
				tp.setTitle("Pick a begin time");
				tp.show();
			}
		});

		final TextView end = (TextView)this.findViewById(R.id.end);
		end.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if (!daily.isChecked()) daily.performClick();

				// get time
				int[] time = getTimeNumbers(end.getText().toString());

				TimePickerDialog tp = new TimePickerDialog(SettingsActivity.this, new TimePickerDialog.OnTimeSetListener() {
					@Override
					public void onTimeSet(TimePicker view, int hourOfDay, int minute) {
						// TODO Auto-generated method stub
						if (minute > 9 && hourOfDay > 9)
							end.setText(""+hourOfDay+":"+minute);
						else if (minute < 9 && hourOfDay < 9)
							end.setText("0"+hourOfDay+":0"+minute);
						else if (minute > 9)
							end.setText("0"+hourOfDay+":"+minute);
						else
							end.setText(""+hourOfDay+":0"+minute);
					}
				}, time[0], time[1], true);
				tp.setTitle("Pick a end time");
				tp.show();
			}
		});

		Button save = (Button)this.findViewById(R.id.save);
		save.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	String str_nch = nch.getText().toString();
            	String str_sps = sps.getText().toString();
            	String str_haverage = haverage.getText().toString();
            	String str_dtime = dtime.getText().toString();

            	if (str_nch.length() > 0) {
            		int int_nch = Integer.parseInt(str_nch);
            		sendCmd(new byte[] { 's', 'n', (byte)(int_nch & 0xff) });
            	}

            	if (str_sps.length() > 0) {
            		double dbl_sps = Double.parseDouble(str_sps);
            		long usec = (long)(1000.0/dbl_sps);

            		sendCmd(new byte[] { 's', 's',
            				(byte)(usec & 0xff),
            				(byte)((usec >> 8) & 0xff),
            				(byte)((usec >> 16) & 0xff),
            				(byte)((usec >> 32) & 0xff)});
            	}

            	if (str_dtime.length() > 0) {
            		long long_dtime = Long.parseLong(str_dtime)*1000;

            		sendCmd(new byte[] { 's', 't',
            				(byte)(long_dtime & 0xff),
            				(byte)((long_dtime >> 8) & 0xff),
            				(byte)((long_dtime >> 16) & 0xff),
            				(byte)((long_dtime >> 32) & 0xff)});
            	}

            	if (str_haverage.length() > 0) {
            		int int_haverage = Integer.parseInt(str_haverage);

            		sendCmd(new byte[] { 's', 'm', (byte)(int_haverage & 0xff) });
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

		restartIOManager();

		Handler handler = new Handler();
		handler.postDelayed(new Runnable() {
		  @Override
		  public void run() {
			  sendCmd(new byte[] { 's', 'g' });
		  }
		}, 150);
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
		if (device == null) {
			log.setText("No serial device.");
			return;
		}

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

	public void refreshGUI(int nch, int average, long tick_time_usec, long time_max_msec)
	{
		TextView nchview = (TextView)this.findViewById(R.id.nch);
		TextView sps = (TextView)this.findViewById(R.id.sps);
		TextView dtime = (TextView)this.findViewById(R.id.dtime);
		TextView haverage = (TextView)this.findViewById(R.id.haverage);

		nchview.setText(""+nch);
		sps.setText(""+(long)(1000.0/tick_time_usec));
		dtime.setText(""+(long)(time_max_msec/1000.0));

		switch (average) {
		case 0:
			average = 4;
			break;
		case 1:
			average = 8;
			break;
		case 2:
			average = 16;
			break;
		case 3:
			average = 32;
			break;
		default:
			average = 0;
			break;
		}
		haverage.setText(""+average);
	}
}