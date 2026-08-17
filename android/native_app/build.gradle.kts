plugins {
    id("com.android.application")
}

android {
    namespace = "com.jmengine.app"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.jmengine.app"
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "14.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
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
