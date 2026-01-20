# Troubleshooting Guide

## Error: 401: invalid_client

This error occurs when the Google OAuth Client ID is not properly configured.

### Quick Fix

1. **Check if Client ID is set:**
   - Open `docs/javascripts/auth.js`
   - Look for line 3: `const GOOGLE_CLIENT_ID = 'YOUR_CLIENT_ID_HERE';`
   - If it still says `YOUR_CLIENT_ID_HERE`, you need to set up Google OAuth

2. **Set up Google OAuth (5 minutes):**
   - Follow the guide in `GOOGLE_AUTH_SETUP.md`
   - Get your Client ID from Google Cloud Console
   - Replace `YOUR_CLIENT_ID_HERE` with your actual Client ID

3. **Common Issues:**
   - **Wrong Client ID format**: Should look like `123456789-abcdefg.apps.googleusercontent.com`
   - **Authorized origins not set**: Make sure `http://localhost:8000` (and your production domain) are added
   - **OAuth consent screen not configured**: Complete the consent screen setup first

### Temporary Workaround

If you need to test immediately without setting up OAuth:

1. The code will automatically fall back to manual login mode
2. Click the sign-in button
3. Enter a @tcd.ie email address when prompted
4. This is for testing only - set up proper OAuth for production

### Verify Your Setup

After updating the Client ID:

1. Restart `mkdocs serve` if it's running
2. Hard refresh the page (Ctrl+Shift+R or Cmd+Shift+R)
3. Check the browser console for any errors
4. Try signing in again

### Still Having Issues?

- Check browser console (F12) for detailed error messages
- Verify your Client ID in Google Cloud Console matches what's in `auth.js`
- Ensure authorized JavaScript origins include your current URL
- Make sure you're using a Web Application type OAuth client (not Desktop)
