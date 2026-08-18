package com.jmengine.app;

import com.jmengine.sdk.JMEngineGlesView;
import com.jmengine.sdk.JMEngineNative;
import com.jmengine.sdk.JMEngineCameraScanner;

import android.app.Activity;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.Manifest;
import android.content.pm.PackageManager;
import android.provider.OpenableColumns;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;

public final class MainActivity extends Activity {
    private static final int REQUEST_OPEN_POINT_CLOUD = 1001;

    private JMEngineNative engine;
    private JMEngineGlesView glView;
    private TextView status;
    private JMEngineCameraScanner cameraScanner;

    @Override protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        engine = new JMEngineNative();
        setContentView(createUi());
        updateStatus("JMEngine loaded\n" + JMEngineNative.version());
    }

    private View createUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);

        status = new TextView(this);
        status.setTextSize(13f);
        status.setPadding(dp(10), dp(6), dp(10), dp(6));
        root.addView(status, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        LinearLayout modeRow = new LinearLayout(this);
        modeRow.setGravity(Gravity.CENTER_VERTICAL);
        addButton(modeRow, "旋转", v -> {
            glView.setInteractionMode(JMEngineGlesView.InteractionMode.ORBIT);
            updateStatus("交互: 旋转 / 双指缩放");
        });
        addButton(modeRow, "表面框选", v -> {
            glView.setInteractionMode(JMEngineGlesView.InteractionMode.SURFACE_RECT);
            updateStatus("交互: GLES 表面框选");
        });
        addButton(modeRow, "穿透框选", v -> {
            glView.setInteractionMode(JMEngineGlesView.InteractionMode.THROUGH_RECT);
            updateStatus("交互: GLES 穿透框选");
        });
        root.addView(modeRow, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        glView = new JMEngineGlesView(this, engine);
        glView.setSelectionListener((count, surfaceOnly) ->
                updateStatus((surfaceOnly ? "Surface" : "Through") + " selected=" + count));
        root.addView(glView, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f));

        LinearLayout actionRow1 = new LinearLayout(this);
        addButton(actionRow1, "打开 PLY/OBJ", v -> openPointCloud());
        addButton(actionRow1, "Fit", v -> glView.fitView());
        addButton(actionRow1, "删除选择", v -> {
            boolean ok = engine.deleteSelection();
            glView.notifyModelChanged();
            updateStatus(ok ? "删除完成" : "没有可删除选择");
        });
        addButton(actionRow1, "清除选择", v -> {
            engine.clearSelection();
            glView.clearSelection();
            updateStatus("selection cleared");
        });
        root.addView(actionRow1, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        LinearLayout actionRow2 = new LinearLayout(this);
        addButton(actionRow2, "Undo", v -> {
            engine.undo(); glView.notifyModelChanged(); updateStatus("undo");
        });
        addButton(actionRow2, "Redo", v -> {
            engine.redo(); glView.notifyModelChanged(); updateStatus("redo");
        });
        addButton(actionRow2, "导出 PLY", v -> exportCurrentPly());
        root.addView(actionRow2, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        LinearLayout scanRow = new LinearLayout(this);
        addButton(scanRow, "开始扫描", v -> startEngineScan());
        addButton(scanRow, "停止", v -> { stopEngineScan(); updateScanStatus("scan stopped"); });
        addButton(scanRow, "离线重建", v -> { boolean ok=engine.reconstructScan(); glView.notifyModelChanged(); updateScanStatus(ok?"reconstructed":"reconstruct failed: "+engine.lastError()); });
        root.addView(scanRow, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT));

        return root;
    }

    private void startEngineScan() {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) { requestPermissions(new String[]{Manifest.permission.CAMERA}, 2001); return; }
        stopEngineScan();
        try {
            File dir=getExternalFilesDir(null); if(dir==null)dir=getFilesDir();
            File calibration=new File(dir,"calib.txt"), vocabulary=new File(dir,"vocab.yml.gz");
            if(!engine.initializeScan(calibration.getAbsolutePath(),vocabulary.getAbsolutePath(),1,6)){updateScanStatus("init failed: "+engine.lastError());return;}
            cameraScanner=new JMEngineCameraScanner(this,engine,new JMEngineCameraScanner.Listener(){public void onStatus(String s){runOnUiThread(()->updateScanStatus(s));}public void onFrame(int id){if((id%10)==0)runOnUiThread(()->{glView.notifyModelChanged();updateScanStatus("frame="+id);});}});
            String[] ids=cameraScanner.cameraIds();if(ids.length<2){updateScanStatus("need two Camera2 devices");cameraScanner.close();cameraScanner=null;return;}
            if(!engine.startScan()){updateScanStatus("start failed");cameraScanner.close();cameraScanner=null;return;}
            cameraScanner.start(ids[0],ids[1],1920,1200,50000);
        } catch(Exception e){stopEngineScan();updateScanStatus("scan exception: "+e.getMessage());}
    }

    private void stopEngineScan() {
        if (cameraScanner != null) { cameraScanner.close(); cameraScanner = null; }
        if (engine != null && engine.scanState() == JMEngineNative.SCAN_SCANNING) engine.stopScan();
    }

    private void updateScanStatus(String action){long[]s=engine.scanStatistics();status.setText(action+"\nstate="+engine.scanState()+" submitted="+s[0]+" processed="+s[1]+" replaced="+s[2]+" points="+s[4]);}

    private void addButton(LinearLayout row, String text, View.OnClickListener listener) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextSize(12f);
        button.setOnClickListener(listener);
        row.addView(button, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
    }

    private void openPointCloud() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        intent.putExtra(Intent.EXTRA_MIME_TYPES, new String[]{
                "application/octet-stream", "text/plain", "model/ply", "model/obj"
        });
        startActivityForResult(intent, REQUEST_OPEN_POINT_CLOUD);
    }

    @Override protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQUEST_OPEN_POINT_CLOUD || resultCode != RESULT_OK || data == null) return;
        Uri uri = data.getData(); if (uri == null) return;
        try {
            File local = copyUriToCache(uri);
            boolean ok = engine.loadPointCloud(local.getAbsolutePath());
            if (!ok) { updateStatus("load failed: " + engine.lastError()); return; }
            glView.fitView();
            glView.notifyModelChanged();
            updateStatus("loaded: " + local.getName());
        } catch (Exception e) {
            updateStatus("load exception: " + e.getMessage());
        }
    }

    private File copyUriToCache(Uri uri) throws Exception {
        String suffix = ".ply";
        String displayName = null;
        try (Cursor cursor = getContentResolver().query(uri, new String[]{OpenableColumns.DISPLAY_NAME}, null, null, null)) {
            if (cursor != null && cursor.moveToFirst()) displayName = cursor.getString(0);
        }
        if (displayName != null && displayName.toLowerCase().endsWith(".obj")) suffix = ".obj";
        File out = File.createTempFile("jmengine_import_", suffix, getCacheDir());
        try (InputStream in = getContentResolver().openInputStream(uri); OutputStream os = new FileOutputStream(out)) {
            if (in == null) throw new IllegalStateException("Cannot open selected document");
            byte[] buffer = new byte[1024 * 1024]; int n;
            while ((n = in.read(buffer)) > 0) os.write(buffer, 0, n);
        }
        return out;
    }

    private void exportCurrentPly() {
        File dir = getExternalFilesDir(null); if (dir == null) dir = getFilesDir();
        File out = new File(dir, "jmengine_export.ply");
        if (engine.savePly(out.getAbsolutePath())) {
            updateStatus("saved:\n" + out.getAbsolutePath());
            Toast.makeText(this, "PLY exported", Toast.LENGTH_SHORT).show();
        } else updateStatus("save failed: " + engine.lastError());
    }

    private void updateStatus(String action) {
        if (status == null) return;
        status.setText(action + "\npoints=" + engine.pointCount()
                + "  active=" + engine.activePointCount()
                + "  deleted=" + engine.deletedPointCount());
    }

    private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }

    @Override protected void onResume() { super.onResume(); if (glView != null) glView.onResume(); }
    @Override protected void onPause() { if (glView != null) glView.onPause(); super.onPause(); }
    @Override protected void onDestroy() { stopEngineScan(); if (engine != null) engine.close(); super.onDestroy(); }
}
