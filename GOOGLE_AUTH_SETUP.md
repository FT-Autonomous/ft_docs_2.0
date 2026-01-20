# Google OAuth Setup Guide

This guide explains how to set up Google OAuth authentication for the Formula Trinity Docs landing page.

## Prerequisites

- A Google Cloud Platform (GCP) account
- Access to create OAuth 2.0 credentials

## Steps to Configure Google OAuth

1. **Go to Google Cloud Console**
   - Visit: https://console.cloud.google.com/
   - Create a new project or select an existing one

2. **Enable Google+ API**
   - Navigate to "APIs & Services" > "Library"
   - Search for "Google+ API" and enable it
   - Alternatively, you can use "Google Identity Services" (newer)

3. **Create OAuth 2.0 Credentials**
   - Go to "APIs & Services" > "Credentials"
   - Click "Create Credentials" > "OAuth client ID"
   - If prompted, configure the OAuth consent screen first:
     - Choose "External" (unless you have a Google Workspace)
     - Fill in the required information
     - Add your email as a test user
   - For application type, select "Web application"
   - Add authorized JavaScript origins:
     - `http://localhost:8000` (for local development)
     - `https://docs.formulatrinity.ie` (for production)
   - Add authorized redirect URIs:
     - `http://localhost:8000` (for local development)
     - `https://docs.formulatrinity.ie` (for production)
   - Click "Create"

4. **Get Your Client ID**
   - Copy the Client ID from the credentials page
   - It will look like: `123456789-abcdefghijklmnop.apps.googleusercontent.com`

5. **Update the Configuration**
   - Open `docs/javascripts/auth.js`
   - Find the line: `const GOOGLE_CLIENT_ID = 'YOUR_CLIENT_ID_HERE';`
   - Replace `YOUR_CLIENT_ID_HERE` with your actual Client ID

## Testing

1. **Local Testing**
   - Run `mkdocs serve`
   - Visit `http://localhost:8000`
   - You should see the landing page with the Google sign-in button
   - Try signing in with a @tcd.ie email address

2. **Domain Restriction**
   - The authentication will only allow emails ending with `@tcd.ie`
   - Users with other email domains will see an error message

## Troubleshooting

- **"Invalid client" error**: Make sure your Client ID is correct and the authorized origins include your domain
- **"Access blocked" error**: Check that your email is added as a test user in the OAuth consent screen (for development)
- **Authentication not working**: Ensure the Google API script is loading correctly (check browser console)

## Security Notes

- The Client ID is safe to expose in client-side code
- Email validation happens client-side for UX, but should be verified server-side in production
- Consider implementing server-side session management for production use

## Alternative: Manual Testing Mode

If you don't want to set up Google OAuth immediately, the code includes a fallback that prompts for an email address (for testing purposes only). This will work automatically if the Google API fails to load.
