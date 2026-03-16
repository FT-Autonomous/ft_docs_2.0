(function () {
  "use strict";

  function logError(message, details) {
    try {
      console.error("[Chatbot]", message, details || {});
    } catch (_) {
      // Avoid crashing if console is unavailable
    }
  }

  var container = document.querySelector(".chatbot-page");
  if (!container) {
    logError("Missing .chatbot-page container");
    return;
  }

  var API_URL = container.dataset.apiUrl;
  var messagesEl = document.getElementById("chatbot-messages");
  var form = document.getElementById("chatbot-form");
  var input = document.getElementById("chatbot-input");
  var sendBtn = document.getElementById("chatbot-send");

  if (!API_URL) {
    logError("Missing API URL on container dataset", {
      dataset: container.dataset,
    });
    return;
  }

  if (!messagesEl || !form || !input || !sendBtn) {
    logError("Missing required chatbot DOM elements", {
      messagesEl: !!messagesEl,
      form: !!form,
      input: !!input,
      sendBtn: !!sendBtn,
    });
    return;
  }

  // Thread ID is kept in memory only and cleared on page refresh.
  var threadId = null;

  function appendMessage(role, text) {
    var div = document.createElement("div");
    div.className = "chatbot-msg chatbot-msg--" + role;
    div.textContent = text;
    messagesEl.appendChild(div);
    messagesEl.scrollTop = messagesEl.scrollHeight;
    return div;
  }

  function setLoading(loading) {
    input.disabled = loading;
    sendBtn.disabled = loading;
    sendBtn.textContent = loading ? "..." : "Send";
  }

  async function readErrorBody(res) {
    try {
      return await res.text();
    } catch (err) {
      logError("Failed to read error response body", {
        error: err,
        status: res && res.status,
      });
      return "";
    }
  }

  form.addEventListener("submit", async function (e) {
    e.preventDefault();

    var message = input.value.trim();
    if (!message) return;

    input.value = "";
    setLoading(true);
    appendMessage("user", message);

    var botDiv = appendMessage("assistant", "Thinking\u2026");

    try {
      var requestBody = { message: message, threadId: threadId };

      var res;
      try {
        res = await fetch(API_URL, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(requestBody),
        });
      } catch (fetchErr) {
        logError("Network or fetch failure", {
          error: fetchErr,
          apiUrl: API_URL,
          requestBody: requestBody,
        });
        throw new Error("Network request failed");
      }

      if (!res.ok) {
        var errorBody = await readErrorBody(res);
        logError("Server returned non-OK response", {
          status: res.status,
          statusText: res.statusText,
          body: errorBody,
          apiUrl: API_URL,
          requestBody: requestBody,
        });
        throw new Error("Server returned " + res.status);
      }

      if (!res.body) {
        logError("Response body is missing", {
          status: res.status,
          statusText: res.statusText,
        });
        throw new Error("Response stream unavailable");
      }

      var reader = res.body.getReader();
      var decoder = new TextDecoder();
      var buffer = "";
      var responseText = "";
      var streamFinished = false;

      while (!streamFinished) {
        var result;

        try {
          result = await reader.read();
        } catch (readErr) {
          logError("Failed while reading response stream", {
            error: readErr,
            partialResponse: responseText,
            buffer: buffer,
            threadId: threadId,
          });
          throw new Error("Failed to read response stream");
        }

        if (result.done) break;

        try {
          buffer += decoder.decode(result.value, { stream: true });
        } catch (decodeErr) {
          logError("Failed to decode response chunk", {
            error: decodeErr,
            chunkLength: result.value ? result.value.length : 0,
          });
          throw new Error("Failed to decode server response");
        }

        var lines = buffer.split("\n");
        buffer = lines.pop();

        for (var i = 0; i < lines.length; i++) {
          var line = lines[i];
          if (!line.startsWith("data: ")) continue;

          var payload = line.slice(6);

          if (payload === "[DONE]") {
            streamFinished = true;
            break;
          }

          var data;
          try {
            data = JSON.parse(payload);
          } catch (parseErr) {
            logError("Failed to parse SSE payload as JSON", {
              error: parseErr,
              payload: payload,
              line: line,
            });
            continue;
          }

          if (data.error) {
            logError("Server sent application error", {
              error: data.error,
              payload: data,
              partialResponse: responseText,
              threadId: threadId,
            });
            throw new Error(data.error);
          }

          if (data.threadId) {
            threadId = data.threadId;
          }

          if (data.text) {
            responseText += data.text;
            botDiv.textContent = responseText;
            messagesEl.scrollTop = messagesEl.scrollHeight;
          }
        }
      }

      if (!responseText) {
        logError("No response text received from server", {
          threadId: threadId,
          remainingBuffer: buffer,
        });
        botDiv.textContent = "No response received. Please try again.";
        botDiv.classList.add("chatbot-msg--error");
      }
    } catch (err) {
      logError("Chatbot request failed", {
        error: err,
        message: message,
        threadId: threadId,
      });
      botDiv.textContent = "Error: " + err.message + ". Please try again.";
      botDiv.classList.add("chatbot-msg--error");
    } finally {
      setLoading(false);
      input.focus();
    }
  });
})();