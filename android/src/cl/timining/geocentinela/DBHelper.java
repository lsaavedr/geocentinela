package cl.timining.geocentinela;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

public class DBHelper extends SQLiteOpenHelper
{
	private static final String TAG = "DBHelper";

    private static final int DB_VERSION = 1;
    private static final String DB_NAME = "centinela";

    private static final String DB_CREATE_TABLE_FILELIST =
    		"create table filelist (id integer primary key," +
    	            "name text, status integer default 0)";

    public DBHelper(Context context)
    {
    	super(context, DB_NAME, null, DB_VERSION);
	}

    @Override
	public void onCreate(SQLiteDatabase db)
    {
    	db.execSQL(DB_CREATE_TABLE_FILELIST);
	}

    @Override
	public void onUpgrade(SQLiteDatabase db, int ov, int nv)
    {
		db.execSQL("DROP TABLE IF EXISTS filelist;");

		db.execSQL(DB_CREATE_TABLE_FILELIST);
	}

    public void addFiles(String[] fileList)
    {
    	SQLiteDatabase dbw = getWritableDatabase();

    	for (int i = 0; i < fileList.length; i++) {
    		Log.v(TAG, "filename:"+fileList[i]+":"+fileList[i].length());

    		ContentValues values = new ContentValues();
    		values.put("name", fileList[i]);

    		int update = dbw.update("filelist", values, "name=?", new String[] { fileList[i] });
    		if (update == 0) dbw.replace("filelist", null, values);
    	}    	
    }

    public void rmFile(String name) {
    	SQLiteDatabase dbw = getWritableDatabase();

    	dbw.delete("filelist", "name=?", new String[] { name });
    }

    public Cursor getFilesCursor()
    {
    	SQLiteDatabase db = getReadableDatabase();
    	Cursor find = db.query("filelist",
                new String[] {"id as _id", "name", "status"},
                null, null,
                null, null,
                "id desc");
    	return find;
    }

    public int updateStatus(String filename, int status)
    {
    	SQLiteDatabase dbw = getWritableDatabase();

    	ContentValues values = new ContentValues();
    	values.put("status", status);

    	int update = dbw.update("filelist", values, "name=?", new String[] { filename });
    	return update;
    }

    public int getStatus(String filename)
    {
    	int status = -1;
    	SQLiteDatabase db = getReadableDatabase();
    	Cursor find = db.query("filelist",
                new String[] { "status" },
                "name=?", new String[] { filename },
                null, null,
                "id desc");
 
    	if (find.moveToFirst() && !find.isAfterLast()) {
            do {
            	status = find.getInt(find.getColumnIndex("status"));
            } while (find.moveToNext());
        } find.close();

    	return status;
    }
}