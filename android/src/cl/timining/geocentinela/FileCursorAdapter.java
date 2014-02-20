package cl.timining.geocentinela;

import java.io.File;
import java.io.IOException;

import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CursorAdapter;
import android.widget.TextView;

public class FileCursorAdapter extends CursorAdapter
{
	private static final String TAG = "FileCursorAdapter";
	private Context context;
    private final LayoutInflater inflater;

    int timeout = 1000;

	public FileCursorAdapter(Context context, Cursor cursor)
	{
		super(context, cursor);
		this.context = context;
		this.inflater = LayoutInflater.from(context);
	}

	@Override
	public void bindView(View view, Context context, Cursor cursor) {
		final int id = cursor.getInt(cursor.getColumnIndex("_id"));
		final String name = cursor.getString(cursor.getColumnIndex("name"));
		int _status = cursor.getInt(cursor.getColumnIndex("status"));

		final File file = new File(MainActivityGeoCentinela.dir, name);
		if (_status==0 && file.exists()) {
			_status = 1;
			if (((MainActivityGeoCentinela)this.context).dbHelper!=null)
				((MainActivityGeoCentinela)this.context).dbHelper.updateStatus(name, _status);
		}
		final int status = _status;

		view.setId(id);

		TextView fileName = (TextView)view.findViewById(R.id.filename);
		fileName.setText(name);

		Button getFile = (Button)view.findViewById(R.id.getfile);
        getFile.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

            	if (status == 0) {
                	char[] nameBytes = name.toCharArray();
                	byte[] cmd = new byte[nameBytes.length+1];
                	cmd[0] = 'g';
                	for (int i = 0; i < nameBytes.length; i++) cmd[i+1] = (byte) nameBytes[i];
                	sendCmd(cmd);
                	Log.v(TAG, new String(cmd));
                	Log.v(TAG, "device:"+((MainActivityGeoCentinela)FileCursorAdapter.this.context).device);
            	}
            }
        });

        Button viewFile = (Button)view.findViewById(R.id.viewfile);
        viewFile.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	Intent intent = new Intent(FileCursorAdapter.this.context, GraphActivity.class);
                intent.putExtra("filename", file.getAbsolutePath());

                FileCursorAdapter.this.context.startActivity(intent);
            }
        });

        Button rmFile = (Button)view.findViewById(R.id.rmfile);
        rmFile.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
            	char[] nameBytes = name.toCharArray();
            	byte[] cmd = new byte[nameBytes.length+1];
            	cmd[0] = 'r';
            	for (int i = 0; i < nameBytes.length; i++) cmd[i+1] = (byte) nameBytes[i];
            	sendCmd(cmd);
            	Log.v(TAG, new String(cmd));
            	Log.v(TAG, "device:"+((MainActivityGeoCentinela)FileCursorAdapter.this.context).device);

            	if (((MainActivityGeoCentinela)FileCursorAdapter.this.context).dbHelper!=null)
            		((MainActivityGeoCentinela)FileCursorAdapter.this.context).dbHelper.rmFile(name);
            	if (file.exists()) file.delete();
            	sendCmd(new byte[] { 'l' });
            }
        });

        if (status==1 && file.exists()) {
        	getFile.setEnabled(false);
        	viewFile.setEnabled(true);
        } else {
        	getFile.setEnabled(true);
        	viewFile.setEnabled(false);
        }
	}

	private void sendCmd(byte[] cmd) {
		if (((MainActivityGeoCentinela)this.context).device == null) return;

		try {
			((MainActivityGeoCentinela)this.context).device.write(cmd, timeout);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public View newView(Context context, Cursor cursor, ViewGroup parent) {
		View view = inflater.inflate(R.layout.file_item, parent, false);
		return view;
	}	
}
