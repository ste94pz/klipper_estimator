#[cfg(test)]
use std::sync::{Arc, Mutex};

use reqwest::Method;
use reqwest::blocking::{Client, Response};
use serde::Serialize;
use thiserror::Error;
use url::Url;

#[cfg(test)]
#[derive(Debug, Clone, Eq, PartialEq, Serialize)]
pub struct RequestRecord {
    pub method: String,
    pub path: String,
}

#[derive(Debug, Error)]
pub enum ReadOnlyMoonrakerError {
    #[error("invalid Moonraker URL: {0}")]
    Url(#[from] url::ParseError),
    #[error("Moonraker URL cannot be a base URL")]
    UrlCannotBeBase,
    #[error("read-only Moonraker policy rejected {method} {path}")]
    Rejected { method: Method, path: String },
    #[error("Moonraker request failed: {0}")]
    Request(#[from] reqwest::Error),
}

impl ReadOnlyMoonrakerError {
    pub fn status(&self) -> Option<reqwest::StatusCode> {
        match self {
            Self::Request(error) => error.status(),
            _ => None,
        }
    }
}

/// Moonraker transport which rejects every request outside the integration
/// allowlist before constructing or sending it.
pub struct ReadOnlyMoonrakerClient {
    base_url: Url,
    api_key: Option<String>,
    client: Client,
    #[cfg(test)]
    requests: Arc<Mutex<Vec<RequestRecord>>>,
}

impl ReadOnlyMoonrakerClient {
    pub fn new(source_url: &str, api_key: Option<&str>) -> Result<Self, ReadOnlyMoonrakerError> {
        let mut base_url = Url::parse(source_url)?;
        base_url.set_query(None);
        base_url.set_fragment(None);
        if base_url.cannot_be_a_base() {
            return Err(ReadOnlyMoonrakerError::UrlCannotBeBase);
        }
        Ok(Self {
            base_url,
            api_key: api_key.map(str::to_owned),
            client: Client::new(),
            #[cfg(test)]
            requests: Arc::new(Mutex::new(Vec::new())),
        })
    }

    pub fn get(&self, segments: &[&str]) -> Result<Response, ReadOnlyMoonrakerError> {
        self.get_with_query(segments, &[])
    }

    pub fn get_with_query(
        &self,
        segments: &[&str],
        query: &[(&str, String)],
    ) -> Result<Response, ReadOnlyMoonrakerError> {
        self.send(Method::GET, segments, query, None::<&serde_json::Value>)
    }

    pub fn post_json<T: Serialize + ?Sized>(
        &self,
        segments: &[&str],
        body: &T,
    ) -> Result<Response, ReadOnlyMoonrakerError> {
        self.send(Method::POST, segments, &[], Some(body))
    }

    #[cfg(test)]
    pub fn request_log(&self) -> Vec<RequestRecord> {
        self.requests.lock().expect("request log poisoned").clone()
    }

    fn send<T: Serialize + ?Sized>(
        &self,
        method: Method,
        segments: &[&str],
        query: &[(&str, String)],
        body: Option<&T>,
    ) -> Result<Response, ReadOnlyMoonrakerError> {
        let path = format!("/{}", segments.join("/"));
        if !is_allowed(&method, segments) {
            return Err(ReadOnlyMoonrakerError::Rejected { method, path });
        }

        let mut url = self.base_url.clone();
        {
            let mut url_segments = url
                .path_segments_mut()
                .map_err(|_| ReadOnlyMoonrakerError::UrlCannotBeBase)?;
            url_segments.pop_if_empty();
            url_segments.extend(segments);
        }
        if !query.is_empty() {
            url.query_pairs_mut()
                .extend_pairs(query.iter().map(|(name, value)| (*name, value.as_str())));
        }

        let mut request = self.client.request(method.clone(), url);
        if let Some(api_key) = &self.api_key {
            request = request.header("X-Api-Key", api_key);
        }
        if let Some(body) = body {
            request = request.json(body);
        }
        #[cfg(test)]
        self.requests
            .lock()
            .expect("request log poisoned")
            .push(RequestRecord {
                method: method.to_string(),
                path,
            });
        Ok(request.send()?.error_for_status()?)
    }
}

fn is_allowed(method: &Method, segments: &[&str]) -> bool {
    let exact_get = matches!(
        segments,
        ["server", "info"]
            | ["printer", "info"]
            | ["printer", "objects", "list"]
            | ["server", "history", "list"]
    );
    let gcode_file_get = segments.starts_with(&["server", "files", "gcodes"])
        && segments.len() > 3
        && segments[3..]
            .iter()
            .all(|segment| !segment.is_empty() && *segment != "." && *segment != "..");
    (*method == Method::GET && (exact_get || gcode_file_get))
        || (*method == Method::POST && segments == ["printer", "objects", "query"])
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::time::Duration;

    use super::*;

    fn recording_server(expected: usize) -> (String, mpsc::Receiver<String>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let (sender, receiver) = mpsc::channel();
        std::thread::spawn(move || {
            for _ in 0..expected {
                let (mut stream, _) = listener.accept().unwrap();
                let mut bytes = [0_u8; 4096];
                let count = stream.read(&mut bytes).unwrap();
                let request = String::from_utf8_lossy(&bytes[..count]);
                sender.send(request.lines().next().unwrap().into()).unwrap();
                stream
                    .write_all(
                        b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}",
                    )
                    .unwrap();
            }
        });
        (format!("http://{address}"), receiver)
    }

    #[test]
    fn allowlisted_requests_are_the_only_requests_reaching_the_server() {
        let (url, received) = recording_server(5);
        let client = ReadOnlyMoonrakerClient::new(&url, Some("secret-key")).unwrap();

        client.get(&["server", "info"]).unwrap();
        client.get(&["printer", "info"]).unwrap();
        client.get(&["printer", "objects", "list"]).unwrap();
        client
            .post_json(
                &["printer", "objects", "query"],
                &serde_json::json!({"objects": {"configfile": ["settings"]}}),
            )
            .unwrap();
        client
            .get_with_query(&["server", "history", "list"], &[("limit", "1".into())])
            .unwrap();

        let lines = (0..5)
            .map(|_| received.recv_timeout(Duration::from_secs(1)).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(lines[0], "GET /server/info HTTP/1.1");
        assert_eq!(lines[1], "GET /printer/info HTTP/1.1");
        assert_eq!(lines[2], "GET /printer/objects/list HTTP/1.1");
        assert_eq!(lines[3], "POST /printer/objects/query HTTP/1.1");
        assert_eq!(lines[4], "GET /server/history/list?limit=1 HTTP/1.1");
        assert!(
            client
                .request_log()
                .iter()
                .all(|request| !request.path.contains("secret-key"))
        );
    }

    #[test]
    fn mutation_routes_are_rejected_before_network_io() {
        let (url, received) = recording_server(0);
        let client = ReadOnlyMoonrakerClient::new(&url, None).unwrap();
        let attempts = [
            (Method::POST, vec!["printer", "gcode", "script"]),
            (Method::POST, vec!["machine", "services", "restart"]),
            (Method::POST, vec!["server", "files", "upload"]),
            (Method::DELETE, vec!["server", "history", "delete_job"]),
            (Method::DELETE, vec!["server", "files", "gcodes", "a.gcode"]),
        ];
        for (method, route) in attempts {
            let error = client
                .send(method, &route, &[], None::<&serde_json::Value>)
                .unwrap_err();
            assert!(matches!(error, ReadOnlyMoonrakerError::Rejected { .. }));
        }
        assert!(client.request_log().is_empty());
        assert!(received.try_recv().is_err());
    }

    #[test]
    fn gcode_download_rejects_path_traversal() {
        let (url, _) = recording_server(0);
        let client = ReadOnlyMoonrakerClient::new(&url, None).unwrap();
        assert!(
            client
                .get(&["server", "files", "gcodes", "..", "printer.cfg"])
                .is_err()
        );
        assert!(client.request_log().is_empty());
    }
}
