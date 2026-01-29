// Google OAuth Configuration
const GOOGLE_CLIENT_ID = '607923524663-lrqs6s40j2obiviflshcnafql615qfd9.apps.googleusercontent.com';
const ALLOWED_DOMAIN = '@tcd.ie';

// Check if Client ID is configured
const isClientIdConfigured = GOOGLE_CLIENT_ID && 
                             GOOGLE_CLIENT_ID !== 'YOUR_CLIENT_ID_HERE' && 
                             GOOGLE_CLIENT_ID.length > 10;

// Check if user is authenticated
function checkAuth() {
  const authStatus = localStorage.getItem('ft_docs_authenticated');
  const userEmail = localStorage.getItem('ft_docs_user_email');
  
  if (authStatus === 'true' && userEmail && userEmail.endsWith(ALLOWED_DOMAIN)) {
    showMainContent();
    return true;
  }
  return false;
}

// Show landing page
function showLandingPage() {
  const landingPage = document.getElementById('landing-page');
  const mainContent = document.getElementById('main-content');
  
  if (landingPage) landingPage.style.display = 'block';
  if (mainContent) mainContent.style.display = 'none';
}

// Show main content after authentication
function showMainContent() {
  const landingPage = document.getElementById('landing-page');
  const mainContent = document.getElementById('main-content');
  
  if (landingPage) landingPage.style.display = 'none';
  if (mainContent) mainContent.style.display = 'block';
}

// Initialize Google Sign-In
function initGoogleSignIn() {
  // If Client ID is not configured, use manual login fallback
  if (!isClientIdConfigured) {
    console.warn('Google OAuth Client ID not configured. Using manual login fallback.');
    setupManualLogin();
    showClientIdWarning();
    return;
  }

  if (typeof gapi === 'undefined') {
    // Load Google API script
    const script = document.createElement('script');
    script.src = 'https://accounts.google.com/gsi/client';
    script.async = true;
    script.defer = true;
    script.onload = () => {
      initializeGoogleAuth();
    };
    script.onerror = () => {
      console.error('Failed to load Google API script');
      setupManualLogin();
      showError('Failed to load Google authentication. Using manual login.');
    };
    document.head.appendChild(script);
  } else {
    initializeGoogleAuth();
  }
}

// Initialize Google Auth
function initializeGoogleAuth() {
  if (typeof google === 'undefined' || !google.accounts) {
    console.error('Google API not loaded');
    // Fallback: Show manual login button
    setupManualLogin();
    return;
  }

  try {
    google.accounts.id.initialize({
      client_id: GOOGLE_CLIENT_ID,
      callback: handleCredentialResponse,
      auto_select: false,
      cancel_on_tap_outside: true
    });
  } catch (error) {
    console.error('Error initializing Google Auth:', error);
    showError('Failed to initialize Google authentication. Please check your Client ID configuration.');
    setupManualLogin();
    return;
  }

  // Render the sign-in button
  const signInButton = document.getElementById('g-signin-button');
  if (signInButton) {
    signInButton.addEventListener('click', () => {
      // Directly use OAuth2 popup flow for more reliable behavior
      try {
        const tokenClient = google.accounts.oauth2.initTokenClient({
          client_id: GOOGLE_CLIENT_ID,
          scope: 'email profile',
          callback: handleTokenResponse,
        });
        // Request access token with consent prompt to ensure fresh login
        tokenClient.requestAccessToken({ prompt: 'consent' });
      } catch (error) {
        console.error('Error initializing OAuth2 client:', error);
        showError('Failed to initialize Google sign-in. Please refresh the page and try again.');
      }
    });
  }
}

// Handle credential response
function handleCredentialResponse(response) {
  if (!response.credential) {
    // Check for error in response
    if (response.error) {
      let errorMessage = 'Authentication failed. ';
      if (response.error === 'invalid_client' || response.error.includes('401')) {
        errorMessage += 'Invalid Client ID. Please check your Google OAuth configuration.';
        showClientIdWarning();
      } else {
        errorMessage += 'Please try again.';
      }
      showError(errorMessage);
      console.error('OAuth error:', response.error);
    } else {
      showError('Authentication failed. Please try again.');
    }
    return;
  }

  // Decode JWT token to get user info
  try {
    const payload = JSON.parse(atob(response.credential.split('.')[1]));
    const email = payload.email;

    if (!email || !email.endsWith(ALLOWED_DOMAIN)) {
      showError('Access denied. Only @tcd.ie email addresses are allowed.');
      return;
    }

    // Store authentication
    localStorage.setItem('ft_docs_authenticated', 'true');
    localStorage.setItem('ft_docs_user_email', email);
    localStorage.setItem('ft_docs_user_name', payload.name || email);

    // Show main content
    showMainContent();
  } catch (error) {
    console.error('Error decoding token:', error);
    showError('Authentication error. Please try again.');
  }
}

// Handle token response (fallback)
function handleTokenResponse(tokenResponse) {
  console.log('Token response received:', tokenResponse);
  
  // Check if response is an error object
  if (!tokenResponse || typeof tokenResponse !== 'object') {
    showError('Invalid response from Google. Please try again.');
    console.error('Invalid token response:', tokenResponse);
    return;
  }
  
  if (tokenResponse.error) {
    let errorMessage = 'Authentication failed. ';
    if (tokenResponse.error === 'invalid_client' || tokenResponse.error === '401') {
      errorMessage += 'Invalid Client ID. Please check your Google OAuth configuration in auth.js.';
      showClientIdWarning();
    } else if (tokenResponse.error === 'popup_closed_by_user' || tokenResponse.error === 'popup_closed') {
      errorMessage = 'Sign-in was cancelled. Please try again.';
    } else if (tokenResponse.error === 'access_denied') {
      errorMessage = 'Access was denied. Please grant the necessary permissions and try again.';
    } else if (tokenResponse.error === 'popup_blocked') {
      errorMessage = 'Popup was blocked. Please allow popups for this site and try again.';
    } else {
      errorMessage += `Error: ${tokenResponse.error}. Please try again.`;
    }
    showError(errorMessage);
    console.error('OAuth error:', tokenResponse.error, tokenResponse);
    return;
  }

  if (!tokenResponse.access_token) {
    showError('Authentication failed. No access token received. Please try again.');
    console.error('No access token in response:', tokenResponse);
    return;
  }

  // Fetch user info using the access token
  fetch('https://www.googleapis.com/oauth2/v2/userinfo', {
    headers: {
      'Authorization': `Bearer ${tokenResponse.access_token}`
    }
  })
  .then(response => {
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return response.json();
  })
  .then(data => {
    console.log('User info received:', data);
    const email = data.email;

    if (!email) {
      showError('Unable to retrieve email address. Please try again.');
      return;
    }

    if (!email.endsWith(ALLOWED_DOMAIN)) {
      showError(`Access denied. Only @tcd.ie email addresses are allowed. Your email (${email}) does not match.`);
      // Clear any partial authentication
      localStorage.removeItem('ft_docs_authenticated');
      localStorage.removeItem('ft_docs_user_email');
      localStorage.removeItem('ft_docs_user_name');
      return;
    }

    // Store authentication
    localStorage.setItem('ft_docs_authenticated', 'true');
    localStorage.setItem('ft_docs_user_email', email);
    localStorage.setItem('ft_docs_user_name', data.name || email);

    // Show main content
    showMainContent();
  })
  .catch(error => {
    console.error('Error fetching user info:', error);
    showError('Authentication error. Please check your connection and try again.');
  });
}

// Show warning about Client ID configuration
function showClientIdWarning() {
  const errorDiv = document.getElementById('error-message');
  if (errorDiv && !isClientIdConfigured) {
    errorDiv.innerHTML = `
      <strong>⚠️ Google OAuth not configured</strong><br>
      To enable Google sign-in, please:<br>
      1. Get a Client ID from <a href="https://console.cloud.google.com/apis/credentials" target="_blank" style="color: #fbbf24;">Google Cloud Console</a><br>
      2. Update <code>GOOGLE_CLIENT_ID</code> in <code>docs/javascripts/auth.js</code><br>
      <br>
      <em>For now, you can use the manual login option below.</em>
    `;
    errorDiv.style.display = 'block';
  }
}

// Manual login fallback (for development/testing)
function setupManualLogin() {
  const signInButton = document.getElementById('g-signin-button');
  if (signInButton) {
    // Update button text to indicate manual mode
    const buttonText = signInButton.querySelector('span');
    if (buttonText) {
      buttonText.textContent = isClientIdConfigured 
        ? 'Sign in with Google' 
        : 'Sign in (Manual Mode - OAuth not configured)';
    }
    
    signInButton.addEventListener('click', () => {
      if (!isClientIdConfigured) {
        // Manual login for testing
        const email = prompt('Enter your @tcd.ie email address (for testing):');
        if (email && email.endsWith(ALLOWED_DOMAIN)) {
          localStorage.setItem('ft_docs_authenticated', 'true');
          localStorage.setItem('ft_docs_user_email', email);
          showMainContent();
        } else if (email) {
          showError('Access denied. Only @tcd.ie email addresses are allowed.');
        }
      } else {
        // OAuth should work, but if we're here, something went wrong
        showError('Google authentication is not available. Please check the console for errors.');
      }
    });
  }
}

// Show error message
function showError(message) {
  const errorDiv = document.getElementById('error-message');
  if (errorDiv) {
    errorDiv.textContent = message;
    errorDiv.style.display = 'block';
    setTimeout(() => {
      errorDiv.style.display = 'none';
    }, 5000);
  }
}

// Logout function
function logout() {
  localStorage.removeItem('ft_docs_authenticated');
  localStorage.removeItem('ft_docs_user_email');
  localStorage.removeItem('ft_docs_user_name');
  showLandingPage();
}

// Inject landing page HTML
function injectLandingPage() {
  // Check if landing page already exists
  if (document.getElementById('landing-page')) {
    return;
  }

  const body = document.body;
  const landingHTML = `
    <div id="landing-page" style="display: none;">
      <div class="landing-container">
        <div class="landing-content">
          <div class="landing-header">
            <h1 class="landing-title">Formula Trinity</h1>
            <p class="landing-subtitle">Documentation Hub</p>
          </div>
          
          <div class="circuit-container">
            <svg class="silverstone-circuit" viewBox="0 0 800 600" xmlns="http://www.w3.org/2000/svg">
              <!-- Silverstone Circuit Track (simplified layout) -->
              <path id="circuit-path" 
                    d="M 150 300 
                        Q 120 200 200 150 
                        L 350 150 
                        Q 450 150 500 200 
                        Q 550 250 600 300 
                        Q 650 350 600 450 
                        Q 550 520 450 550 
                        Q 350 570 250 550 
                        Q 150 520 120 450 
                        Q 100 400 120 350 
                        Q 130 320 150 300 Z" 
                    fill="none" 
                    stroke="currentColor" 
                    stroke-width="45" 
                    stroke-linecap="round" 
                    stroke-linejoin="round"
                    class="circuit-track"/>
              <path id="circuit-center" 
                    d="M 150 300 
                        Q 120 200 200 150 
                        L 350 150 
                        Q 450 150 500 200 
                        Q 550 250 600 300 
                        Q 650 350 600 450 
                        Q 550 520 450 550 
                        Q 350 570 250 550 
                        Q 150 520 120 450 
                        Q 100 400 120 350 
                        Q 130 320 150 300 Z" 
                    fill="none" 
                    stroke="currentColor" 
                    stroke-width="2" 
                    stroke-dasharray="8,4"
                    class="circuit-line"/>
              
              <!-- Racing Car -->
              <g id="racing-car" transform="translate(150, 300)">
                <circle cx="0" cy="0" r="10" fill="#dc2626" opacity="0.15"/>
                <rect x="-14" y="-7" width="28" height="14" fill="#dc2626" rx="2"/>
                <rect x="-10" y="-12" width="20" height="7" fill="#991b1b" rx="1"/>
                <circle cx="-7" cy="9" r="6" fill="#1a1a1a"/>
                <circle cx="7" cy="9" r="6" fill="#1a1a1a"/>
                <circle cx="-7" cy="9" r="3" fill="#4a4a4a"/>
                <circle cx="7" cy="9" r="3" fill="#4a4a4a"/>
                <rect x="-7" y="-5" width="14" height="4" fill="#fbbf24" opacity="0.95"/>
              </g>
            </svg>
          </div>

          <div class="login-section">
            <div class="login-card">
              <p class="login-description">Sign in with your Trinity College Dublin account</p>
              <div id="g-signin-button" class="google-signin-button">
                <svg class="google-icon" viewBox="0 0 24 24">
                  <path fill="#4285F4" d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"/>
                  <path fill="#34A853" d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"/>
                  <path fill="#FBBC05" d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"/>
                  <path fill="#EA4335" d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"/>
                </svg>
                <span>Continue with Google</span>
              </div>
              <p class="login-note">@tcd.ie email required</p>
              <div id="error-message" class="error-message" style="display: none;"></div>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;
  body.insertAdjacentHTML('afterbegin', landingHTML);
}

// Show landing page
function showLandingPage() {
  injectLandingPage();
  const landingPage = document.getElementById('landing-page');
  const mainContent = document.querySelector('.md-container');
  
  if (landingPage) {
    landingPage.style.display = 'block';
    // Hide the main content
    if (mainContent) {
      mainContent.style.display = 'none';
    }
    // Also hide navigation
    const nav = document.querySelector('.md-header');
    if (nav) nav.style.display = 'none';
    const sidebar = document.querySelector('.md-sidebar');
    if (sidebar) sidebar.style.display = 'none';
  }
}

// Show main content after authentication
function showMainContent() {
  const landingPage = document.getElementById('landing-page');
  const mainContent = document.querySelector('.md-container');
  
  if (landingPage) landingPage.style.display = 'none';
  if (mainContent) mainContent.style.display = 'block';
  
  // Show navigation
  const nav = document.querySelector('.md-header');
  if (nav) nav.style.display = '';
  const sidebar = document.querySelector('.md-sidebar');
  if (sidebar) sidebar.style.display = '';
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
  // Check if we're on the index page
  const isIndexPage = window.location.pathname === '/' || 
                      window.location.pathname === '/index.html' || 
                      window.location.pathname.endsWith('/') ||
                      window.location.pathname.includes('index');
  
  if (isIndexPage) {
    if (!checkAuth()) {
      showLandingPage();
      initGoogleSignIn();
      startCarAnimation();
    } else {
      showMainContent();
    }
  }
});

// Racing Car Animation - Silverstone Circuit
function startCarAnimation() {
  const car = document.getElementById('racing-car');
  const path = document.getElementById('circuit-path');
  if (!car || !path) return;

  const pathLength = path.getTotalLength();
  let progress = 0;
  const speed = 0.003; // Adjust speed (0.001 = slow, 0.01 = fast)
  
  function getPointAtLength(length) {
    return path.getPointAtLength(length);
  }
  
  function getAngleAtLength(length) {
    const point1 = path.getPointAtLength(length);
    const point2 = path.getPointAtLength((length + 1) % pathLength);
    return Math.atan2(point2.y - point1.y, point2.x - point1.x) * (180 / Math.PI);
  }

  function animate() {
    progress = (progress + speed) % 1;
    const currentLength = progress * pathLength;
    const point = getPointAtLength(currentLength);
    const angle = getAngleAtLength(currentLength);
    
    car.setAttribute('transform', `translate(${point.x}, ${point.y}) rotate(${angle + 90})`);
    requestAnimationFrame(animate);
  }

  animate();
}
