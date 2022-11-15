from behave import *




def before_all(context):
    context.USB_VENDOR_ID="16c0"
    context.USB_PRODUCT_ID="05e1"
    context.USB_SERIAL_TEST="TEST_123456789"
    
