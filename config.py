# AWS endpoint parameters.
HOST = b'a35lkm5jyds64h-ats'    # ex: b'abcdefg1234567'
REGION = b'us-east-2'  # ex: b'us-east-1'

CLIENT_ID = "UPS-demo"  # Should be unique for each device connected.
AWS_ENDPOINT = b'%s.iot.%s.amazonaws.com' % (HOST, REGION)

WIFI_SSID = 'Willowbrae'
WIFI_PASSWORD = 'wbca1234'

KEYFILE = '/private.pem.key'
CERTFILE = '/certificate.pem.crt'