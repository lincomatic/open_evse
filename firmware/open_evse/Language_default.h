//Language pack for OpenEVSE

//#ifndef __Language__
//#define __Language__

// For making your own language make a copy of this file with name
// "language_yourlanguage.h" and include file in "open_evse.h" under language heading
// Remember LCD is only 16 characters wide and no special characters

//**********************************************************
//English default language
//**********************************************************

#define STR_SETTINGS	    			"Settings"
#define STR_SETUP					      "Setup"
#define STR_SERVICE_LEVEL			  "Service Level"
#define STR_MAX_CURRENT			  	"Max Current"
#define STR_DIODE_CHECK				  "Diode Check"
#define STR_VENT_REQD_CHECK			"Vent Req'd Check"
#define STR_BACKLIGHT_TYPE			"Backlight Type"
#define STR_GROUND_CHECK			  "Ground Check"
#define STR_STUCK_RELAY_CHK			"Stuck Relay Chk"
#define STR_GFI_SELF_TEST			  "GFI Self Test"
#define STR_TEMPERATURE_CHK			"Temperature Chk"
#define STR_HIGH_TEMP			"HIGH TEMP"
#define STR_YES_NO					    {"Yes", "No"}
#define STR_RESTART_NOW         "Restart Now?"
#define STR_RESTART					    "Restart"
#define STR_EXIT					      "Exit"
#define STR_DATE_TIME				    "Date/Time"
#define STR_MONTH				      	"Month"
#define STR_DAY					        "Day"
#define STR_YEAR                "Year"
#define STR_HOUR                "Hour"
#define STR_MINUTE					    "Minute"
#define STR_DELAY_TIMER				  "Delay Timer"
#define STR_START_HOUR			    "Start Hour"
#define STR_START_MIN				    "Start Min"
#define STR_STOP_HOUR				    "Stop Hour"
#define STR_STOP_MIN				    "Stop Min"
#define STR_CHARGE_LIMIT	  		"Charge Limit"
#define STR_TIME_LIMIT			  	"Time Limit"
#define STR_RGB_MONOCHROME		  {"RGB","Monochrome"}
#define STR_POWER_ON		        "Power On"
#define STR_SELF_TEST				    "Self Test"
#define STR_AUTO_DETECT			  	"Auto Detect"
#define STR_SVC_LEVEL_L1		  	"Svc Level: L1"
#define STR_SVC_LEVEL_L2		  	"Svc Level: L2"
#define STR_TEST_FAILED			  	"TEST FAILED"
#define STR_EVSE_ERROR			  	"EVSE ERROR"
#define STR_SERVICE_REQUIRED		"SERVICE REQUIRED"
#define STR_VENT_REQUIRED			  "VENT REQUIRED"
#define STR_DIODE_CHECK_FAILED	"DIODE CHECK"
#define STR_GFCI_FAULT				  "GFCI FAULT"
#define STR_GFCI					      "GFCI"
#define STR_RETRY_IN				    "RETRY IN "
#define STR_OVER_TEMPERATURE		"OVER TEMPERATURE"
#define STR_NO_GROUND				    "NO GROUND"
#define STR_STUCK_RELAY				  "STUCK RELAY"
#define STR_DISABLED				   "Disabled"
#define STR_RESETTING                              "Restarting..."

#define STR_WAITING					   "Waiting"
#define STR_SLEEPING				   "Sleeping"
#define STR_CONNECTED			     "Connected"
#define STR_TEST_DISABLED		   "TEST DISABLED"
#define STRF_L_A					     "L%d:%dA"
#define STR_READY					     "Ready"
#define STR_CHARGING				   "Charging"
#define STRF_MAX_CURRENT			 "%s Max Current"
#define STR_SET_DATE_TIME			 "Set Date/Time?"
#define STR_YESNO_SETSTART_SETSTOP	{"Yes/No","Set Start","Set Stop"}
#define STRF_WH						     "%5luWh"  
#define STRF_KWH					     "%6lukWh" 
#define STRF_VOLT              " %3luV"
#define STR_AUTO					     "Auto" 
#define STR_LEVEL_1					   "Level 1 (120V)" 
#define STR_LEVEL_2					   "Level 2 (240V)"  
#define STR_L1						     "L1"  
#define STR_L2						     "L2"  
#define STR_OVER_CURRENT "OVERCURRENT"
//#endif
