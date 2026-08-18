plugins {
    id("com.android.application")
}

android {
    namespace = "com.jmengine.app"
    compileSdk {
        version = release(37)
    }

    signingConfigs {
        create("releaseDebugKey") {
            storeFile = file("${System.getProperty("user.home")}/.android/debug.keystore")
            storePassword = "android"
            keyAlias = "androiddebugkey"
            keyPassword = "android"
        }
    }

    defaultConfig {
        applicationId = "com.jmengine.app"
        minSdk = 24
        targetSdk = 37
        versionCode = 1
        versionName = "20.1"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            signingConfig = signingConfigs.getByName("releaseDebugKey")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

dependencies {
    implementation(project(":jmengine"))
}
